## Warning ⚠️
The main branch is locked and no one should attempt to push directly to the main branch

## Process
* Create a new branch from the main branch (make sure the branch is up to date)
    * make sure you are on the main branch and pull down new changes
        * `git checkout main`
        * `git pull`
            * if there is any merge conflict please make sure you are merging the right commits with your local branch
    * create a new branch
        * `git checkout -b <feature-name/ issue>`
        * This will automatically put you into the new branch so you don't have to do anything
            * to check your current branch use the following command:
            * `git branch`
            
* Build out your feature as you want
    * 🙅🏼 Make sure run the following commands to make sure there are no conflicts with the main branch 🙅🏼
    * `git checkout main`
    * `git pull`
    * `git checkout <branch name>`
    * `git merge main`
    * this makes sure that the main is up to date and locally merges the main changes to your branch so there is no issue when we are creating a pull request
* Create a [PR (pull request)](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests)
    * The pull request body should include the following
        * purpose of the feature / what it accomplishes
        * how to test the feature
        * if the pull requests solves any problem under the issue tab, please use `#<issue number>` to tag it
    * Have another member of the team run the code locally to make sure there are no errors or conflict 

* Merge the branch into the local branch and enjoy 👋🏼
