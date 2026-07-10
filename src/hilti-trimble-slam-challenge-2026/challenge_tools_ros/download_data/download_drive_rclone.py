#!/usr/bin/env python3
"""
Google Drive Folder Downloader (rclone version)

Downloads all files and folders from a Google Drive folder using rclone.

Setup:
    1. Install rclone:
       macOS:   brew install rclone
       Linux:   sudo apt install rclone  (or: sudo dnf install rclone)
       Windows: choco install rclone  (or: winget install rclone)

    2. Configure Google Drive remote (one-time):
       rclone config
       - Choose "n" for new remote
       - Name it "gdrive"
       - Choose "drive" (Google Drive)
       - Leave client_id and client_secret blank
       - Choose "1" for full access
       - Follow browser auth flow

Usage:
    python download_drive_rclone.py
    python download_drive_rclone.py -f FOLDER_ID -o ./output
"""

import argparse
import subprocess
import shutil
import sys
from pathlib import Path

# The folder ID from the URL
# https://drive.google.com/drive/u/3/folders/1BWFIfEL40Nvj-yeyre5O9dOiYCTWatv5
DEFAULT_FOLDER_ID = "1BWFIfEL40Nvj-yeyre5O9dOiYCTWatv5"


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Download files from a public Google Drive folder using rclone"
    )
    parser.add_argument(
        "--folder-id",
        "-f",
        default=DEFAULT_FOLDER_ID,
        help=f"Google Drive folder ID (default: {DEFAULT_FOLDER_ID})",
    )
    parser.add_argument(
        "--output",
        "-o",
        default="./challenge_data",
        help="Output directory (default: ./challenge_data)",
    )
    args = parser.parse_args()

    # Check if rclone is installed
    if not shutil.which("rclone"):
        print("ERROR: rclone is not installed!")
        print()
        print("Install with:")
        print("  macOS:   brew install rclone")
        print("  Linux:   sudo apt install rclone")
        print("  Windows: choco install rclone")
        print()
        sys.exit(1)

    # Check if gdrive remote is configured
    result = subprocess.run(
        ["rclone", "listremotes"],
        capture_output=True,
        text=True,
    )
    if "gdrive:" not in result.stdout:
        print("ERROR: rclone 'gdrive' remote not configured!")
        print()
        print("Run 'rclone config' and create a remote named 'gdrive':")
        print("  1. Choose 'n' for new remote")
        print("  2. Name it 'gdrive'")
        print("  3. Choose 'drive' (Google Drive)")
        print("  4. Leave client_id and client_secret blank")
        print("  5. Choose '1' for full access")
        print("  6. Follow the browser auth flow")
        print()
        sys.exit(1)

    folder_id = args.folder_id
    output_dir = Path(args.output)

    print("=" * 60)
    print("Google Drive Folder Downloader (rclone)")
    print("=" * 60)
    print()

    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)

    # Build rclone source path using configured gdrive remote
    source = f"gdrive,root_folder_id={folder_id}:"

    print(f"Folder ID: {folder_id}")
    print(f"Output directory: {output_dir.absolute()}")
    print()
    print("Starting download...")
    print("-" * 60)

    # Run rclone copy
    cmd = [
        "rclone",
        "copy",
        source,
        str(output_dir),
        "--progress",
        "--transfers=4",
        "--drive-acknowledge-abuse",  # Download even if flagged
    ]

    try:
        subprocess.run(cmd, check=True)
        print("-" * 60)
        print("Download complete!")
        print(f"  Location: {output_dir.absolute()}")
    except subprocess.CalledProcessError as e:
        print("-" * 60)
        print(f"Download failed with exit code {e.returncode}")
        sys.exit(e.returncode)


if __name__ == "__main__":
    main()
