import { BlobReadable } from "@mcap/browser";
import { McapIndexedReader, type Schema } from "@mcap/core";
import { loadDecompressHandlers } from "@mcap/support";
import { parse } from "@foxglove/rosmsg";
import { MessageReader } from "@foxglove/rosmsg2-serialization";

type McapTopicEntry = {
    alias: string;
    topic: string;
    kind: "pointcloud2" | "topological_map" | "generic";
    role: "pointcloud" | "graph" | "generic";
    message_type: string;
    message_encoding: string;
    schema_encoding: string;
    row_count: number;
    sample_stride: number;
    messages: Array<{ stamp_ns: number; data: unknown }>;
    unsupported_reason?: string;
};

export type McapViewerBundle = {
    version: "topo_mcap_browser_v1";
    source: {
        file_name: string;
        file_size: number;
        profile: string;
        library: string;
        start_time_ns: number;
        end_time_ns: number;
    };
    topics: Record<string, McapTopicEntry>;
};

const MAX_POINT_FRAME_NUM = 300;
const MAX_GRAPH_FRAME_NUM = 300;
const MAX_TOTAL_POINT_NUM = 2000000;
const MAX_POINT_NUM_PER_FRAME = 50000;

function classifySchema(schemaName: string): Pick<McapTopicEntry, "kind" | "role"> {
    const name = schemaName.toLowerCase();
    if (name.endsWith("/pointcloud2") || name.endsWith("::pointcloud2")) {
        return { kind: "pointcloud2", role: "pointcloud" };
    }
    if (name.endsWith("/topologicalmap") || name.includes("topological_map")) {
        return { kind: "topological_map", role: "graph" };
    }
    return { kind: "generic", role: "generic" };
}

function makeMessageReader(schema: Schema): MessageReader | undefined {
    if (schema.encoding !== "ros2msg") {
        return undefined;
    }
    const schemaText = new TextDecoder().decode(schema.data);
    return new MessageReader(parse(schemaText, { ros2: true }));
}

function finiteNumber(value: unknown, fallback = 0): number {
    if (typeof value === "bigint") {
        return Number(value);
    }
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : fallback;
}

function fieldValue(
    view: DataView,
    offset: number,
    datatype: number,
    isLittleEndian: boolean,
): number | undefined {
    if (offset < 0 || offset >= view.byteLength) {
        return undefined;
    }
    switch (datatype) {
        case 1:
            return view.getInt8(offset);
        case 2:
            return view.getUint8(offset);
        case 3:
            return view.getInt16(offset, isLittleEndian);
        case 4:
            return view.getUint16(offset, isLittleEndian);
        case 5:
            return view.getInt32(offset, isLittleEndian);
        case 6:
            return view.getUint32(offset, isLittleEndian);
        case 7:
            return view.getFloat32(offset, isLittleEndian);
        case 8:
            return view.getFloat64(offset, isLittleEndian);
        default:
            return undefined;
    }
}

function decodePointCloud2(
    message: unknown,
    maxPointNum: number,
): { points: Array<[number, number, number]>; frame_id: string } {
    const source = message as {
        header?: { frame_id?: string };
        height?: number;
        width?: number;
        fields?: Array<{ name?: string; offset?: number; datatype?: number }>;
        is_bigendian?: boolean;
        point_step?: number;
        row_step?: number;
        data?: Uint8Array | number[];
    };
    const width = Math.max(0, Math.floor(finiteNumber(source.width)));
    const height = Math.max(1, Math.floor(finiteNumber(source.height, 1)));
    const pointStep = Math.max(0, Math.floor(finiteNumber(source.point_step)));
    const rowStep = Math.max(pointStep * width, Math.floor(finiteNumber(source.row_step)));
    const rawData = source.data instanceof Uint8Array ? source.data : new Uint8Array(source.data ?? []);
    const fields = new Map((source.fields ?? []).map((field) => [String(field.name ?? ""), field]));
    const xField = fields.get("x");
    const yField = fields.get("y");
    const zField = fields.get("z");
    if (!xField || !yField || !zField || pointStep <= 0 || rawData.byteLength < pointStep) {
        throw new Error("PointCloud2のx/y/z fieldまたはdataが不正です");
    }

    const pointNum = width * height;
    const sampleStride = Math.max(1, Math.ceil(pointNum / maxPointNum));
    const view = new DataView(rawData.buffer, rawData.byteOffset, rawData.byteLength);
    const isLittleEndian = !source.is_bigendian;
    const points: Array<[number, number, number]> = [];
    for (let pointIdx = 0; pointIdx < pointNum; pointIdx += sampleStride) {
        const rowIdx = Math.floor(pointIdx / width);
        const columnIdx = pointIdx % width;
        const baseOffset = rowIdx * rowStep + columnIdx * pointStep;
        const x = fieldValue(view, baseOffset + finiteNumber(xField.offset), finiteNumber(xField.datatype), isLittleEndian);
        const y = fieldValue(view, baseOffset + finiteNumber(yField.offset), finiteNumber(yField.datatype), isLittleEndian);
        const z = fieldValue(view, baseOffset + finiteNumber(zField.offset), finiteNumber(zField.datatype), isLittleEndian);
        if (x !== undefined && y !== undefined && z !== undefined && [x, y, z].every(Number.isFinite)) {
            points.push([x, y, z]);
        }
    }
    return { points, frame_id: String(source.header?.frame_id ?? "") };
}

function normalizeDecodedValue(value: unknown): unknown {
    if (typeof value === "bigint") {
        return Number(value);
    }
    if (ArrayBuffer.isView(value) && !(value instanceof DataView)) {
        return Array.from(value as unknown as ArrayLike<number>);
    }
    if (Array.isArray(value)) {
        return value.map(normalizeDecodedValue);
    }
    if (value && typeof value === "object") {
        return Object.fromEntries(
            Object.entries(value).map(([key, child]) => [key, normalizeDecodedValue(child)]),
        );
    }
    return value;
}

export async function readFile(file: File): Promise<McapViewerBundle> {
    const decompressHandlers = await loadDecompressHandlers();
    const reader = await McapIndexedReader.Initialize({
        readable: new BlobReadable(file),
        decompressHandlers,
        messageIndexCacheSizeBytes: 8 * 1024 * 1024,
    });
    const topics: Record<string, McapTopicEntry> = {};
    const entriesByChannelId = new Map<number, McapTopicEntry>();
    const messageReaders = new Map<number, MessageReader>();
    const readableTopics = new Set<string>();

    for (const channel of reader.channelsById.values()) {
        const schema = reader.schemasById.get(channel.schemaId);
        const schemaName = schema?.name ?? "";
        const classification = classifySchema(schemaName);
        const alias = topics[channel.topic] ? `${channel.topic}#${channel.id}` : channel.topic;
        const rowCount = Number(reader.statistics?.channelMessageCounts.get(channel.id) ?? 0n);
        const maxFrameNum = classification.kind === "pointcloud2" ? MAX_POINT_FRAME_NUM : MAX_GRAPH_FRAME_NUM;
        const entry: McapTopicEntry = {
            alias,
            topic: channel.topic,
            ...classification,
            message_type: schemaName,
            message_encoding: channel.messageEncoding,
            schema_encoding: schema?.encoding ?? "",
            row_count: rowCount,
            sample_stride: rowCount > 0 ? Math.max(1, Math.ceil(rowCount / maxFrameNum)) : 1,
            messages: [],
        };
        topics[alias] = entry;
        entriesByChannelId.set(channel.id, entry);
        if (!schema || channel.messageEncoding !== "cdr" || classification.kind === "generic") {
            entry.unsupported_reason = classification.kind === "generic"
                ? "表示対象外のROS 2 message型"
                : "ros2msg/CDR以外のschemaまたはmessage encoding";
            continue;
        }
        try {
            const messageReader = makeMessageReader(schema);
            if (!messageReader) {
                entry.unsupported_reason = "ros2msg schemaではありません";
                continue;
            }
            messageReaders.set(channel.id, messageReader);
            readableTopics.add(channel.topic);
        } catch (error) {
            entry.unsupported_reason = `message定義の解析失敗: ${error instanceof Error ? error.message : String(error)}`;
        }
    }

    if (readableTopics.size > 0) {
        const seenMessageNumByChannelId = new Map<number, number>();
        for await (const record of reader.readMessages({ topics: [...readableTopics] })) {
            const channel = reader.channelsById.get(record.channelId);
            const entry = entriesByChannelId.get(record.channelId);
            const messageReader = messageReaders.get(record.channelId);
            if (!channel || !entry || !messageReader) {
                continue;
            }
            const seenMessageNum = seenMessageNumByChannelId.get(record.channelId) ?? 0;
            seenMessageNumByChannelId.set(record.channelId, seenMessageNum + 1);
            const maxFrameNum = entry.kind === "pointcloud2" ? MAX_POINT_FRAME_NUM : MAX_GRAPH_FRAME_NUM;
            if (seenMessageNum % entry.sample_stride !== 0 || entry.messages.length >= maxFrameNum) {
                continue;
            }
            try {
                const decoded = messageReader.readMessage(record.data);
                const storedFrameNum = entry.row_count > 0
                    ? Math.min(maxFrameNum, Math.ceil(entry.row_count / entry.sample_stride))
                    : maxFrameNum;
                const maxPointNum = Math.max(
                    1,
                    Math.min(MAX_POINT_NUM_PER_FRAME, Math.floor(MAX_TOTAL_POINT_NUM / storedFrameNum)),
                );
                const data = entry.kind === "pointcloud2"
                    ? decodePointCloud2(decoded, maxPointNum)
                    : normalizeDecodedValue(decoded);
                entry.messages.push({ stamp_ns: Number(record.logTime), data });
            } catch (error) {
                entry.unsupported_reason = `message復号失敗: ${error instanceof Error ? error.message : String(error)}`;
            }
        }
    }

    const statistics = reader.statistics;
    return {
        version: "topo_mcap_browser_v1",
        source: {
            file_name: file.name,
            file_size: file.size,
            profile: reader.header.profile,
            library: reader.header.library,
            start_time_ns: Number(statistics?.messageStartTime ?? 0n),
            end_time_ns: Number(statistics?.messageEndTime ?? 0n),
        },
        topics,
    };
}
