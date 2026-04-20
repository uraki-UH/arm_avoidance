import { PointCloudData } from '../types';
import { generateUUID } from './uuid';

const LANDXML_NS = 'http://www.landxml.org/schema/LandXML-1.2';

/**
 * Parse LandXML TIN surface and extract all points.
 * This loader reads the first Surface element and extracts all point coordinates.
 * 
 * Note: Unlike the Python version, this does not perform any decimation -
 * all points are loaded for display.
 * 
 * Coordinates are centered around the origin for proper display,
 * as survey coordinates (northing/easting) are typically very large values.
 */
export async function loadLandXML(file: File): Promise<PointCloudData> {
    const text = await file.text();
    const parser = new DOMParser();
    const doc = parser.parseFromString(text, 'application/xml');

    // Check for parse errors
    const parseError = doc.querySelector('parsererror');
    if (parseError) {
        throw new Error('Invalid XML file: ' + parseError.textContent);
    }

    // Find the first Surface element (try with and without namespace)
    let surface = doc.getElementsByTagNameNS(LANDXML_NS, 'Surface')[0];
    if (!surface) {
        // Try without namespace (some LandXML files may not use namespace)
        surface = doc.getElementsByTagName('Surface')[0];
    }
    if (!surface) {
        throw new Error('No Surface element found in LandXML file');
    }

    // Find Pnts element
    let pnts = surface.getElementsByTagNameNS(LANDXML_NS, 'Pnts')[0];
    if (!pnts) {
        pnts = surface.getElementsByTagName('Pnts')[0];
    }
    if (!pnts) {
        throw new Error('No Pnts element found in Surface');
    }

    // Extract all P elements
    let pElements = pnts.getElementsByTagNameNS(LANDXML_NS, 'P');
    if (pElements.length === 0) {
        pElements = pnts.getElementsByTagName('P');
    }

    if (pElements.length === 0) {
        throw new Error('No points found in LandXML file');
    }

    const pointCount = pElements.length;
    const points = new Float32Array(pointCount * 3);

    // First pass: Parse all points and calculate centroid
    let sumX = 0, sumY = 0, sumZ = 0;
    let validCount = 0;

    for (let i = 0; i < pElements.length; i++) {
        const pElement = pElements[i];
        const text = pElement.textContent?.trim();
        if (!text) continue;

        const coords = text.split(/\s+/).map(parseFloat);
        if (coords.length >= 3 && !isNaN(coords[0]) && !isNaN(coords[1]) && !isNaN(coords[2])) {
            // LandXML typically stores coordinates as "northing easting elevation" (Y X Z)
            // Store temporarily before centering
            points[i * 3] = coords[0];
            points[i * 3 + 1] = coords[1];
            points[i * 3 + 2] = coords[2];

            sumX += coords[0];
            sumY += coords[1];
            sumZ += coords[2];
            validCount++;
        }
    }

    // Calculate centroid
    if (validCount > 0) {
        const centroidX = sumX / validCount;
        const centroidY = sumY / validCount;
        const centroidZ = sumZ / validCount;

        // Second pass: Center coordinates around origin
        for (let i = 0; i < pointCount; i++) {
            points[i * 3] -= centroidX;
            points[i * 3 + 1] -= centroidY;
            points[i * 3 + 2] -= centroidZ;
        }

        console.log(`LandXML: Centered ${validCount} points from centroid (${centroidX.toFixed(2)}, ${centroidY.toFixed(2)}, ${centroidZ.toFixed(2)})`);
    }

    // Get surface name for display
    const surfaceName = surface.getAttribute('name') || 'LandXML Surface';

    return {
        id: generateUUID(),
        name: `${file.name} - ${surfaceName}`,
        points,
        count: pointCount
    };
}

