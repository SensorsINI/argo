#!/usr/bin/env python3
"""
Foxglove Layout Validation Script
Validates Foxglove layout JSON files for proper structure and format.
"""

import json
import sys
import os
from pathlib import Path

def validate_foxglove_layout(layout_path):
    """Validate a Foxglove layout JSON file."""
    try:
        with open(layout_path, 'r') as f:
            layout = json.load(f)
        
        # Check required top-level fields
        required_fields = [
            'configById', 'globalVariables', 'userNodes', 
            'linkedGlobalVariables', 'playbackConfig', 'layout'
        ]
        
        missing_fields = [field for field in required_fields if field not in layout]
        if missing_fields:
            print(f"❌ Missing required fields: {missing_fields}")
            return False
        
        # Check linkedGlobalVariables is a list
        if not isinstance(layout['linkedGlobalVariables'], list):
            print("❌ linkedGlobalVariables must be a list")
            return False
        
        # Check layout structure
        if not isinstance(layout['layout'], dict):
            print("❌ layout must be a dictionary")
            return False
        
        # Check configById structure
        if not isinstance(layout['configById'], dict):
            print("❌ configById must be a dictionary")
            return False
        
        # Validate panel IDs in layout match configById
        def check_panel_ids(obj, path=""):
            if isinstance(obj, dict):
                for key, value in obj.items():
                    current_path = f"{path}.{key}" if path else key
                    if key in ['first', 'second'] and isinstance(value, str):
                        # This should be a panel ID
                        if '!' not in value:
                            print(f"❌ Invalid panel ID format at {current_path}: {value}")
                            return False
                        if value not in layout['configById']:
                            print(f"❌ Panel ID {value} not found in configById")
                            return False
                    elif isinstance(value, (dict, str)):
                        if not check_panel_ids(value, current_path):
                            return False
            return True
        
        if not check_panel_ids(layout['layout']):
            return False
        
        print("✅ Result: VALID")
        return True
        
    except json.JSONDecodeError as e:
        print(f"❌ JSON parsing error: {e}")
        return False
    except Exception as e:
        print(f"❌ Validation error: {e}")
        return False

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 debug_foxglove_layout.py <layout_file.json>")
        sys.exit(1)
    
    layout_path = sys.argv[1]
    if not os.path.exists(layout_path):
        print(f"❌ File not found: {layout_path}")
        sys.exit(1)
    
    print(f"Validating Foxglove layout: {layout_path}")
    success = validate_foxglove_layout(layout_path)
    
    if not success:
        sys.exit(1)

if __name__ == "__main__":
    main()
