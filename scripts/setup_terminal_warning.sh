#!/bin/bash
# Setup script to add Argo storage warning to terminal

SCRIPT_DIR="/home/orangepi/argo/scripts"
WARNING_SCRIPT="$SCRIPT_DIR/argo_terminal_warning.sh"

# Make the warning script executable
chmod +x "$WARNING_SCRIPT"

# Function to add to bashrc if not already present
add_to_bashrc() {
    local line="$1"
    local file="$2"
    
    if ! grep -Fq "$line" "$file"; then
        echo "" >> "$file"
        echo "# Argo storage warning" >> "$file"
        echo "$line" >> "$file"
        echo "Added Argo storage warning to $file"
    else
        echo "Argo storage warning already present in $file"
    fi
}

# Add to .bashrc
if [[ -f "$HOME/.bashrc" ]]; then
    add_to_bashrc "source $WARNING_SCRIPT" "$HOME/.bashrc"
fi

# Add to .profile (for login shells)
if [[ -f "$HOME/.profile" ]]; then
    add_to_bashrc "source $WARNING_SCRIPT" "$HOME/.profile"
fi

echo "✅ Argo terminal storage warning setup complete!"
echo "   The warning will appear in new terminal sessions."
echo "   To test immediately, run: source $WARNING_SCRIPT"
echo ""
echo "To remove the warning later, edit ~/.bashrc and ~/.profile"
echo "and remove the lines containing 'argo_terminal_warning.sh'"
