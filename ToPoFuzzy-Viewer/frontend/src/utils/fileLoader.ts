import { PointCloudData } from '../types';
import { loadPCD } from './pcdLoader';
import { loadLAS } from './lasLoader';
import { loadPLY } from './plyLoader';
import { loadLandXML } from './landxmlLoader';

export async function loadPointCloudFile(file: File): Promise<PointCloudData> {
    const ext = file.name.split('.').pop()?.toLowerCase();

    switch (ext) {
        case 'pcd':
            return loadPCD(file);
        case 'las':
        case 'laz':
            return loadLAS(file);
        case 'ply':
            return loadPLY(file);
        case 'xml':
        case 'landxml':
        case 'landxm':
            return loadLandXML(file);
        default:
            throw new Error(`Unsupported file format: ${ext}`);
    }
}
