# Mobile Robotics
Implement an Invariant EKF for a high speed racing unmanned aerial vehicle (UAV)


# Contributing

Clone repo with SSH:

```
$ git clone git@github.com:ghaggin/mobile-robotics.git 
```

or HTTPS:

```
$ git clone https://github.com/ghaggin/mobile-robotics.git
```

## Workflow

### Keeping master up to date
Do not make changes on master.  To keep master up to date with repo perform the following

```
$ git checkout master
$ git fetch origin
$ git rebase origin/master
```

### Adding a feature

To add a new feature, create a new branch, make changes to that branch, push those changes to the remote, submit a merge request.  Members of the project will review the changes and merge the change into the master branch.
```
$ git checkout -b <new-branch-name>
```

Whenever changes to the master branch is made, update the feature branch
```
$ git fetch origin
$ git rebase origin/master
```
If any of your feature branch changes confilict with master branch changes, solve all merge conflicts.

To create add changes to a feature branch to master, commit changes and push to remote
```
$ git add -A
$ git commit -m "Descriptive commit message"
$ git push origin <branch-name>
```

At this point, go to github and submit a pull request with the branch that you just pushed.