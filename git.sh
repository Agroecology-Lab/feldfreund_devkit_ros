#!/bin/bash

# --- 1. CONFIGURATION ---
REPO_URL="https://github.com/Agroecology-Lab/feldfreund_devkit_ros.git"

# Default branch
BRANCH="main"

# Check if the first argument is a branch flag (starts with -)
if [[ "$1" == -* ]]; then
    # Strip the hyphen to get the branch name (e.g., -sowbot becomes sowbot)
    BRANCH="${1#-}"
    shift
    MESSAGE="$*"
else
    MESSAGE="$*"
fi

# --- 2. VALIDATION ---
if [ -z "$MESSAGE" ]; then
    echo "Error: No commit message provided."
    echo "Usage: ./git.sh [-branchname] your message here"
    echo "Example: ./git.sh -sowbot added seeding logic"
    exit 1
fi

echo "--------------------------------------------"
echo "Target Repo:   $REPO_URL"
echo "Target Branch: $BRANCH"
echo "Message:       $MESSAGE"
echo "--------------------------------------------"

# --- 3. REMOTE SYNC ---
# Ensure the remote 'origin' matches the target repo
CURRENT_REMOTE=$(git remote get-url origin 2>/dev/null)
if [ "$CURRENT_REMOTE" != "$REPO_URL" ]; then
    echo "Updating git remote origin to $REPO_URL"
    git remote set-url origin "$REPO_URL" || git remote add origin "$REPO_URL"
fi

# --- 4. BRANCH MANAGEMENT ---
if ! git rev-parse --verify "$BRANCH" >/dev/null 2>&1; then
    echo "Creating new local branch: $BRANCH"
    git checkout -b "$BRANCH"
else
    echo "Switching to branch: $BRANCH"
    git checkout "$BRANCH"
fi

# --- 5. GIT WORKFLOW ---
echo "Adding changes..."
git add .

echo "Committing..."
git commit -m "$MESSAGE" || echo "Nothing new to commit."

# --- 6. SYNC LOGIC ---
if git ls-remote --exit-code --heads origin "$BRANCH" >/dev/null 2>&1; then
    echo "Branch exists on GitHub. Pulling and Rebasing..."
    if ! git pull origin "$BRANCH" --rebase; then
        echo "ERROR: Conflict detected during pull! Fix manually then run: git rebase --continue"
        exit 1
    fi
else
    echo "Branch not on GitHub yet. Skipping pull..."
fi

echo "Pushing to origin..."
if git push -u origin "$BRANCH"; then
    echo "--------------------------------------------"
    echo "SUCCESS: Your changes are now on GitHub!"
    echo "URL: ${REPO_URL%.git}/tree/$BRANCH"
    echo "--------------------------------------------"
else
    echo "Error: Push failed."
    exit 1
fi
