import os
import shutil
import sys

def get_size(path):
    """Calculate size of file or directory in bytes."""
    if os.path.islink(path):
        return 0
    try:
        if os.path.isfile(path):
            return os.path.getsize(path)
        
        total_size = 0
        with os.scandir(path) as it:
            for entry in it:
                if entry.is_symlink():
                    continue
                if entry.is_file():
                    total_size += entry.stat().st_size
                elif entry.is_dir():
                    # Stay away from virtual filesystems
                    if entry.name in ['proc', 'sys', 'dev', 'run']:
                        continue
                    total_size += get_size(entry.path)
        return total_size
    except (PermissionError, FileNotFoundError, OSError):
        return 0

def format_bytes(size):
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if size < 1024:
            return f"{size:.2f} {unit}"
        size /= 1024
    return f"{size:.2f} PB"

def scan_path(target_path, limit=12):
    print(f"\n🔍 Detailed Scan: {target_path}")
    print(f"{'Folder/File':<35} | {'Size':<10}")
    print("-" * 50)
    
    items = []
    try:
        with os.scandir(target_path) as it:
            for entry in it:
                size = get_size(entry.path)
                items.append((entry.name, size))
    except Exception as e:
        print(f"Error accessing {target_path}: {e}")
        return

    items.sort(key=lambda x: x[1], reverse=True)
    for name, size in items[:limit]:
        if size > 1024 * 1024: # Only show items > 1MB for clarity
            print(f"{name[:35]:<35} | {format_bytes(size):<10}")

if __name__ == "__main__":
    # Check partition health first
    total, used, free = shutil.disk_usage("/")
    print(f"📊 Disk Summary (Root /): Free: {format_bytes(free)}")

    # Specific targets for Sam's ThinkPad
    if os.getuid() != 0:
        print("⚠️  Run as 'sudo' to accurately scan /var/lib/docker")
    
    scan_path("/var")
    scan_path("/var/lib")
    if os.path.exists("/var/lib/docker"):
        scan_path("/var/lib/docker")
