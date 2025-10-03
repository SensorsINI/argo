#!/usr/bin/env python3
"""
Convert KMZ files to GeoJSON format for Foxglove visualization
"""

import zipfile
import xml.etree.ElementTree as ET
import json
import sys
import os
from pathlib import Path

def parse_kml_to_geojson(kml_content):
    """Convert KML content to GeoJSON format"""
    root = ET.fromstring(kml_content)
    
    # Define namespaces
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    
    features = []
    
    # Find all Placemarks
    for placemark in root.findall('.//kml:Placemark', ns):
        name_elem = placemark.find('kml:name', ns)
        name = name_elem.text if name_elem is not None else "unnamed"
        
        # Check for Point (waypoints)
        point = placemark.find('.//kml:Point', ns)
        if point is not None:
            coords_elem = point.find('kml:coordinates', ns)
            if coords_elem is not None:
                coords = coords_elem.text.strip()
                lon, lat, alt = coords.split(',')
                
                feature = {
                    "type": "Feature",
                    "properties": {
                        "name": name,
                        "type": "waypoint"
                    },
                    "geometry": {
                        "type": "Point",
                        "coordinates": [float(lon), float(lat), float(alt)]
                    }
                }
                features.append(feature)
        
        # Check for LineString (sailing boundaries)
        line = placemark.find('.//kml:LineString', ns)
        if line is not None:
            coords_elem = line.find('kml:coordinates', ns)
            if coords_elem is not None:
                coords_text = coords_elem.text.strip()
                coords_list = []
                for coord in coords_text.split():
                    lon, lat, alt = coord.split(',')
                    coords_list.append([float(lon), float(lat), float(alt)])
                
                feature = {
                    "type": "Feature",
                    "properties": {
                        "name": name,
                        "type": "sailing_boundary"
                    },
                    "geometry": {
                        "type": "LineString",
                        "coordinates": coords_list
                    }
                }
                features.append(feature)
        
        # Check for Polygon (sailing areas and hazards)
        polygon = placemark.find('.//kml:Polygon', ns)
        if polygon is not None:
            outer_boundary = polygon.find('.//kml:outerBoundaryIs/kml:LinearRing', ns)
            if outer_boundary is not None:
                coords_elem = outer_boundary.find('kml:coordinates', ns)
                if coords_elem is not None:
                    coords_text = coords_elem.text.strip()
                    coords_list = []
                    for coord in coords_text.split():
                        lon, lat, alt = coord.split(',')
                        coords_list.append([float(lon), float(lat), float(alt)])
                    
                    # Close the polygon if not already closed
                    if coords_list[0] != coords_list[-1]:
                        coords_list.append(coords_list[0])
                    
                    feature_type = "hazard" if "rock" in name.lower() else "sailing_area"
                    
                    feature = {
                        "type": "Feature",
                        "properties": {
                            "name": name,
                            "type": feature_type
                        },
                        "geometry": {
                            "type": "Polygon",
                            "coordinates": [coords_list]
                        }
                    }
                    features.append(feature)
    
    geojson = {
        "type": "FeatureCollection",
        "features": features
    }
    
    return geojson

def convert_kmz_to_geojson(kmz_path, output_path):
    """Convert a KMZ file to GeoJSON"""
    print(f"Converting {kmz_path} to {output_path}")
    
    with zipfile.ZipFile(kmz_path, 'r') as kmz:
        # Find the KML file (usually doc.kml)
        kml_files = [f for f in kmz.namelist() if f.endswith('.kml')]
        if not kml_files:
            print(f"Error: No KML file found in {kmz_path}")
            return False
        
        # Read the first KML file
        kml_content = kmz.read(kml_files[0]).decode('utf-8')
        
        # Convert to GeoJSON
        geojson = parse_kml_to_geojson(kml_content)
        
        # Write output
        with open(output_path, 'w') as f:
            json.dump(geojson, f, indent=2)
        
        print(f"Successfully converted to {output_path}")
        print(f"Found {len(geojson['features'])} features")
        
        # Print feature summary
        for feature in geojson['features']:
            props = feature['properties']
            geom_type = feature['geometry']['type']
            print(f"  - {props['name']} ({props['type']}, {geom_type})")
        
        return True

def main():
    maps_dir = Path("/home/orangepi/argo/maps")
    output_dir = Path("/home/orangepi/argo/foxglove/maps")
    
    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Find all KMZ files
    kmz_files = list(maps_dir.glob("*.kmz"))
    
    if not kmz_files:
        print("No KMZ files found in maps directory")
        return
    
    print(f"Found {len(kmz_files)} KMZ files to convert")
    
    for kmz_file in kmz_files:
        output_file = output_dir / f"{kmz_file.stem}.geojson"
        convert_kmz_to_geojson(kmz_file, output_file)
        print()

if __name__ == "__main__":
    main()
