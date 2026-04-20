#pragma once

#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <string>

namespace viewer_internal {

using json = nlohmann::json;

inline bool parseJson(const std::string& text, json& out) {
    try {
        out = json::parse(text);
        return true;
    } catch (...) {
        return false;
    }
}

inline std::string makeOkResponse(const std::string& id, const json& result = json::object()) {
    json response;
    response["id"] = id;
    response["ok"] = true;
    response["result"] = result;
    return response.dump();
}

inline std::string makeErrorResponse(const std::string& id,
                                     const std::string& code,
                                     const std::string& message,
                                     const json& details = json::object()) {
    json response;
    response["id"] = id;
    response["ok"] = false;
    response["error"] = {
        {"code", code},
        {"message", message},
        {"details", details}
    };
    return response.dump();
}

inline std_msgs::msg::String toStringMsg(const std::string& text) {
    std_msgs::msg::String msg;
    msg.data = text;
    return msg;
}

inline std_msgs::msg::String toStringMsg(const json& payload) {
    std_msgs::msg::String msg;
    msg.data = payload.dump();
    return msg;
}

inline std::string getMethod(const json& request) {
    if (!request.contains("method") || !request["method"].is_string()) {
        return "";
    }
    return request["method"].get<std::string>();
}

inline std::string getId(const json& request) {
    if (!request.contains("id")) {
        return "";
    }
    if (request["id"].is_string()) {
        return request["id"].get<std::string>();
    }
    if (request["id"].is_number_integer()) {
        return std::to_string(request["id"].get<int64_t>());
    }
    return "";
}

inline const json& getParams(const json& request) {
    static const json empty = json::object();
    if (!request.contains("params") || !request["params"].is_object()) {
        return empty;
    }
    return request["params"];
}

inline bool startsWith(const std::string& value, const std::string& prefix) {
    return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

} // namespace viewer_internal
