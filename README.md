# 2021dev
Learning wpilib

# 2021dev
Learning wpilib

### 1. Branch from devTest to create new features or fix bugs

`git checkout devTest`  
`git checkout -b feature/my-feature` OR `git checkout -b bugfix/my-bugfix`
 
### 2. Merge new feature to devUnstable
  
`git checkout devUnstable`  
`git merge feature/my-feature`

If you have a bug fix, checkout the branch that you want to add your bug fix code to. NOTE: if you have a bug fix for devTest, create a pull request in order to merge.

### 3. Once fully tested in devUnstable, create pull request to devTest

[Link to instructions](https://docs.github.com/en/github/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request) or ask Mr. P
  
Ask someone else to review the code and approve the request.
  
### Check if the code in devTest working
  
## NOTE: Do not branch from main or merge to main!
