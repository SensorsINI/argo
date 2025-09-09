#!/bin/bash
#
# SD Card Cache Flush Script for Armbian
# ======================================
#
# This script provides various methods to flush SD card cache
# to ensure data integrity, especially before power cuts.
#

echo "SD Card Cache Flush Utility"
echo "=========================="

# Function to flush cache
flush_cache() {
    echo "Flushing SD card cache..."
    
    # Method 1: Standard sync
    echo "1. Running sync command..."
    sync
    if [ $? -eq 0 ]; then
        echo "   ✓ sync completed successfully"
    else
        echo "   ✗ sync failed"
    fi
    
    # Method 2: Force flush specific device
    echo "2. Flushing SD card device..."
    # Try to find the SD card device
    SD_DEVICE=""
    for dev in /dev/mmcblk*; do
        if [ -b "$dev" ] && ! [[ "$dev" =~ p[0-9]+$ ]]; then
            SD_DEVICE="$dev"
            break
        fi
    done
    
    if [ -n "$SD_DEVICE" ]; then
        echo "   Found SD card: $SD_DEVICE"
        blockdev --flushbufs "$SD_DEVICE" 2>/dev/null
        if [ $? -eq 0 ]; then
            echo "   ✓ Device flush completed successfully"
        else
            echo "   ✗ Device flush failed (may require sudo)"
        fi
    else
        echo "   ✗ No SD card device found"
    fi
    
    # Method 3: Drop caches (more aggressive)
    echo "3. Dropping page caches..."
    if [ -w /proc/sys/vm/drop_caches ]; then
        echo 3 > /proc/sys/vm/drop_caches 2>/dev/null
        if [ $? -eq 0 ]; then
            echo "   ✓ Page caches dropped successfully"
        else
            echo "   ✗ Failed to drop caches (may require sudo)"
        fi
    else
        echo "   ✗ Cannot access /proc/sys/vm/drop_caches (may require sudo)"
    fi
    
    echo "Cache flush operations completed."
}

# Function to show current cache settings
show_cache_info() {
    echo "Current Cache Settings:"
    echo "======================"
    
    # Show commit interval from fstab
    echo "Filesystem commit interval:"
    grep -E "ext4.*commit=" /etc/fstab | sed 's/.*commit=\([0-9]*\).*/\1 seconds/'
    
    # Show dirty writeback settings
    echo -e "\nDirty writeback settings:"
    echo "dirty_expire_centisecs: $(cat /proc/sys/vm/dirty_expire_centisecs 2>/dev/null || echo 'N/A')"
    echo "dirty_writeback_centisecs: $(cat /proc/sys/vm/dirty_writeback_centisecs 2>/dev/null || echo 'N/A')"
    echo "dirty_ratio: $(cat /proc/sys/vm/dirty_ratio 2>/dev/null || echo 'N/A')"
    echo "dirty_background_ratio: $(cat /proc/sys/vm/dirty_background_ratio 2>/dev/null || echo 'N/A')"
}

# Main script logic
case "${1:-flush}" in
    "flush"|"")
        flush_cache
        ;;
    "info")
        show_cache_info
        ;;
    "help"|"-h"|"--help")
        echo "Usage: $0 [command]"
        echo ""
        echo "Commands:"
        echo "  flush    Flush SD card cache (default)"
        echo "  info     Show current cache settings"
        echo "  help     Show this help message"
        echo ""
        echo "Examples:"
        echo "  $0           # Flush cache"
        echo "  $0 flush     # Flush cache"
        echo "  $0 info      # Show cache info"
        ;;
    *)
        echo "Unknown command: $1"
        echo "Use '$0 help' for usage information"
        exit 1
        ;;
esac
