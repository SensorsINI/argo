#!/bin/bash
# Argo Branch Migration Script
# Migrates from 'standard-ros2-launch' branch to 'dev' branch
#
# Usage:
#   ./migrate_to_dev_branch.sh
#
# This script:
#   1. Fetches the latest changes from remote
#   2. Creates/checks out the new 'dev' branch from remote
#   3. Deletes the old 'standard-ros2-launch' branch locally and remotely (if exists)
#   4. Sets up proper tracking for the new 'dev' branch

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo -e "${BLUE}Argo Branch Migration: standard-ros2-launch -> dev${NC}"
echo ""

# Check if we're in a git repository
if ! git rev-parse --git-dir > /dev/null 2>&1; then
    echo -e "${RED}Error: Not in a git repository${NC}"
    exit 1
fi

# Change to project directory
cd "$PROJECT_DIR"

# Check current branch
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
echo -e "${YELLOW}Current branch: ${CURRENT_BRANCH}${NC}"

# Check if there are uncommitted changes
if ! git diff-index --quiet HEAD --; then
    echo -e "${YELLOW}Warning: You have uncommitted changes${NC}"
    echo -e "${YELLOW}Please commit or stash them before running this script${NC}"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Migration cancelled"
        exit 1
    fi
fi

# Fetch latest changes
echo -e "${BLUE}Fetching latest changes from remote...${NC}"
git fetch origin

# Check if remote dev branch exists
if git ls-remote --heads origin dev | grep -q dev; then
    echo -e "${GREEN}Remote 'dev' branch found${NC}"
else
    echo -e "${RED}Error: Remote 'dev' branch not found${NC}"
    echo -e "${YELLOW}Make sure the branch has been pushed to remote first${NC}"
    exit 1
fi

# If currently on the old branch, switch to master first
if [ "$CURRENT_BRANCH" = "standard-ros2-launch" ]; then
    echo -e "${YELLOW}Currently on old branch 'standard-ros2-launch'${NC}"
    echo -e "${BLUE}Switching to master temporarily...${NC}"
    git checkout master 2>/dev/null || git checkout -b master origin/master 2>/dev/null || {
        echo -e "${RED}Error: Could not switch to master${NC}"
        exit 1
    }
fi

# Delete local old branch if it exists
if git show-ref --verify --quiet refs/heads/standard-ros2-launch; then
    echo -e "${BLUE}Deleting local 'standard-ros2-launch' branch...${NC}"
    git branch -D standard-ros2-launch
    echo -e "${GREEN}Local old branch deleted${NC}"
else
    echo -e "${YELLOW}Local 'standard-ros2-launch' branch not found (already deleted)${NC}"
fi

# Checkout or create the dev branch
if git show-ref --verify --quiet refs/heads/dev; then
    echo -e "${BLUE}Local 'dev' branch already exists, checking it out...${NC}"
    git checkout dev
    echo -e "${BLUE}Updating 'dev' branch from remote...${NC}"
    git pull origin dev
else
    echo -e "${BLUE}Creating and checking out 'dev' branch from remote...${NC}"
    git checkout -b dev origin/dev
    echo -e "${GREEN}Created and checked out 'dev' branch${NC}"
fi

# Set up tracking
echo -e "${BLUE}Setting up branch tracking...${NC}"
git branch --set-upstream-to=origin/dev dev
echo -e "${GREEN}Branch tracking configured${NC}"

# Clean up remote tracking references
echo -e "${BLUE}Cleaning up remote tracking references...${NC}"
git fetch --prune origin
if git ls-remote --heads origin standard-ros2-launch | grep -q standard-ros2-launch; then
    echo -e "${YELLOW}Note: Remote 'standard-ros2-launch' branch still exists on remote${NC}"
    echo -e "${YELLOW}It will be deleted separately by the repository maintainer${NC}"
fi

# Final status
echo ""
echo -e "${GREEN}✓ Migration complete!${NC}"
echo ""
echo -e "${BLUE}Current status:${NC}"
git status

echo ""
echo -e "${GREEN}You are now on the 'dev' branch${NC}"
echo -e "${GREEN}The old 'standard-ros2-launch' branch has been removed locally${NC}"

