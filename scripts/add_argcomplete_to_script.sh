#!/bin/bash
# Helper script to add argcomplete to a Python script with argparse
# Usage: ./add_argcomplete_to_script.sh path/to/script.py

set -e

if [ $# -ne 1 ]; then
    echo "Usage: $0 <python_script.py>"
    echo ""
    echo "This script adds argcomplete support to a Python script that uses argparse."
    echo "It will:"
    echo "  1. Add 'PYTHON_ARGCOMPLETE_OK' marker (for global argcomplete)"
    echo "  2. Add 'import argcomplete' after argparse import"
    echo "  3. Add 'argcomplete.autocomplete(parser)' before parse_args()"
    echo ""
    echo "Example: $0 nodes/controller.py"
    exit 1
fi

SCRIPT_FILE="$1"

if [ ! -f "$SCRIPT_FILE" ]; then
    echo "❌ Error: File not found: $SCRIPT_FILE"
    exit 1
fi

# Check if argparse is imported
if ! grep -q "import argparse" "$SCRIPT_FILE"; then
    echo "❌ Error: Script does not import argparse"
    echo "This script only works with scripts that use argparse"
    exit 1
fi

# Check if PYTHON_ARGCOMPLETE_OK marker is present
if grep -q "PYTHON_ARGCOMPLETE_OK" "$SCRIPT_FILE"; then
    echo "✅ PYTHON_ARGCOMPLETE_OK marker already present"
else
    echo "Adding 'PYTHON_ARGCOMPLETE_OK' marker..."
    # Add marker after shebang line (line 2)
    sed -i '1a # PYTHON_ARGCOMPLETE_OK' "$SCRIPT_FILE"
    echo "✅ Added marker"
fi

# Check if argcomplete is already imported
if grep -q "import argcomplete" "$SCRIPT_FILE"; then
    echo "✅ argcomplete already imported"
else
    echo "Adding 'import argcomplete'..."
    # Add import after argparse import
    sed -i '/^import argparse$/a import argcomplete' "$SCRIPT_FILE"
    echo "✅ Added import"
fi

# Check if autocomplete is already called
if grep -q "argcomplete.autocomplete" "$SCRIPT_FILE"; then
    echo "✅ argcomplete.autocomplete() already present"
else
    echo "Adding 'argcomplete.autocomplete(parser)' before parse_args()..."
    # Add autocomplete call before parse_args()
    # This handles "parser.parse_args()" with any arguments inside
    sed -i '/\.parse_args(/i \    # Enable bash completion for command-line arguments\n    argcomplete.autocomplete(parser)' "$SCRIPT_FILE"
    echo "✅ Added autocomplete call"
fi

echo ""
echo "🎉 Successfully added argcomplete to $SCRIPT_FILE"
echo ""
echo "To test tab completion:"
echo "  1. Reload bash: source ~/.bashrc"
echo "  2. Try either:"
echo "     - python3 $SCRIPT_FILE --<TAB>"
echo "     - ./$SCRIPT_FILE --<TAB>  (if script is executable)"

