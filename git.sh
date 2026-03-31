#!/bin/bash
set -e  # Exit immediately if any command fails

REPO_URL="https://github.com/Agroecology-Lab/feldfreund_devkit_ros.git"
BRANCH="main"

# Improved branch detection: 
# If $1 starts with '-' OR if $1 is a known branch name, use it.
if [[ "$1" == -* ]]; then
    BRANCH="${1#-}"
    shift
    MESSAGE="$*"
elif git rev-parse --verify "$1" >/dev/null 2>&1; then
    BRANCH="$1"
    shift
    MESSAGE="$*"
else
    MESSAGE="$*"
fi

if [ -z "$MESSAGE" ]; then
    echo "Error: No commit message."
    exit 1
fi

echo "--> Target: $BRANCH | Message: $MESSAGE"

# Ensure remote is correct
git remote set-url origin "$REPO_URL" 2>/dev/null || git remote add origin "$REPO_URL"

# Switch branch - will now EXIT if it fails (e.g. due to uncommitted changes)
if ! git rev-parse --verify "$BRANCH" >/dev/null 2>&1; then
    git checkout -b "$BRANCH"
else
    git checkout "$BRANCH"
fi

echo "Adding and Committing..."
git add .
git commit -m "$MESSAGE" || echo "Nothing new to commit."

# Sync
if git ls-remote --exit-code --heads origin "$BRANCH" >/dev/null 2>&1; then
    echo "Syncing with GitHub..."
    git pull origin "$BRANCH" --rebase
fi

echo "Pushing..."
git push -u origin "$BRANCH"
