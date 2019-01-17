# Team 1675 git Guide

## Commands
In general you should only use the below commands in git. If you have a problem or need to do something besides the stuff listed below ask a mentor.

- `git status`
  - Reports your repository's current status, most importantly shows staged, unstaged, and untracked files/changes.
- `git checkout branch-name`
  - Switches to an existing branch named `branch-name`.
- `git checkout -b new-branch`
  - Creates a new branch off of the current branch named `new-branch`.
- `git add -u`
  - Adds all unstaged changes from existing tracked files to your repository.
- `git add path/to/file.md`
  - Adds a file that was untracked to be tracked by your repository.
- `git commit -m "message"`
  - Commits your staged changes to your local repository with the commit message "message".
- `git push origin new-branch`
  - Pushes any commits from `new-branch` in your machine to origin's (github's) `new-branch`.
- `git fetch`
  - Get all changes from `origin` that you don't have and store them locally for use.
- `git merge branch-name`
  - Merges all changes that your branch doesn't have from `branch-name` to your branch.

## Typical Workflow
1. `git fetch`
1. `git checkout origin/master`
1. `git checkout -b my-branch`
1. Make code changes, test
1. `git add -u`
1. `git commit -m "My message"`
1. `git fetch`
1. `git merge origin/master`
1. `git push origin my-branch`
1. Make PR, review with team
1. Make and push additional commits to fix review comments
