#!/bin/bash

# --- 1. CONFIGURATION ---
if [ "$1" == "-dev" ]; then
    BRANCH="dev"
    shift
    MESSAGE="$*"
else
    BRANCH="main"
    MESSAGE="$*"
fi

# --- 2. VALIDATION ---
if [ -z "$MESSAGE" ]; then
    echo "❌ Error: No commit message provided."
    echo "Usage: ./git.sh [-dev] your message here"
    exit 1
fi

echo "--------------------------------------------"
echo "🛠️  Target Branch: $BRANCH"
echo "💬 Message: $MESSAGE"
echo "--------------------------------------------"

# --- 3. BRANCH MANAGEMENT ---
# Check if branch exists, if not create it
if ! git rev-parse --verify $BRANCH >/dev/null 2>&1; then
    echo "🌿 Creating new local branch: $BRANCH"
    git checkout -b $BRANCH
else
    echo "🌿 Switching to branch: $BRANCH"
    git checkout $BRANCH
fi

# --- 4. GIT WORKFLOW ---
echo "📦 Adding changes..."
git add .

echo "📝 Committing..."
# We allow this to fail if there's nothing new to commit
git commit -m "$MESSAGE" || echo "⚠️  Nothing new to commit."

# --- 5. SYNC LOGIC (The "Fix") ---
# Check if the branch exists on the server (remote)
if git ls-remote --exit-code --heads origin $BRANCH >/dev/null 2>&1; then
    echo "🔄 Branch exists on GitHub. Pulling & Rebasing..."
    if ! git pull origin $BRANCH --rebase; then
        echo "❌ ERROR: Conflict detected during pull! Fix manually then run: git rebase --continue"
        exit 1
    fi
else
    echo "🆕 Branch not on GitHub yet. Skipping pull..."
fi

echo "🚀 Pushing to origin..."
# -u sets the upstream so future 'git push' works without arguments
if git push -u origin $BRANCH; then
    echo "--------------------------------------------"
    echo "✨ SUCCESS: Your changes are now on GitHub!"
    echo "🔗 URL: https://github.com/Agroecology-Lab/Open_agbot_devkit_ros/tree/$BRANCH"
    echo "--------------------------------------------"
else
    echo "❌ Error: Push failed. Check your internet or GitHub token."
    exit 1
fi
