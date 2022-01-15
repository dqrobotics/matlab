# Welcome contributors to the DQ Robotics!

We are happy about your interest in our project. Thank you for your time. DQ Robotics is a standalone open-source library and your contributions are always welcome!.

This is a set of guidelines for contribuiting to [DQ Robotics](https://dqrobotics.github.io/).


...

# Workflow

## Case 1 (Common cases)

- If your modifications pass all tests, Prof. Bruno Adorno and Prof. Murilo Marinho will review all the changes and they will accept them after all necessary adjustments if any. 

## Case 2 (Very rare cases)
- In some cases, your modifications would fail some tests because of incompatibility with the current version of [matlab-tests](https://github.com/dqrobotics/matlab-tests) and/or [matlab-examples](https://github.com/dqrobotics/matlab-examples). In thoses cases, you would propose changes in [matlab-tests](https://github.com/dqrobotics/matlab-tests) and [matlab-examples](https://github.com/dqrobotics/matlab-examples) to make them compatible with your new version of the dqrobotics/matlab. 
-  Prof. Bruno Adorno and Prof. Murilo Marinho will review the changes proposed in both [matlab-tests](https://github.com/dqrobotics/matlab-tests) and [matlab-examples](https://github.com/dqrobotics/matlab-examples). They will accept the modifications in the master branch after all necessary adjustments if any. 
-  Then, they will test your dqrobotics/matlab pull request again. They will make adjustments if any until your dqrobotics/matlab pull request passes all the tests. After that, they will review deeply your modifications.
-  Finally, Prof. Bruno Adorno and Prof. Murilo Marinho will accept your modifications in the master branch.


# Example

## Fork the [dqrobotics/matlab](https://github.com/dqrobotics/matlab) in your Github account

![fork_master](https://user-images.githubusercontent.com/23158313/149602838-133f6c09-2e16-418e-8ab6-47fb36a91056.gif)

## Clone the forked repository

![git_clone](https://user-images.githubusercontent.com/23158313/149603381-78732b55-2794-4be9-9a12-b7062d0649b5.gif)


## Do your modifications
![modifications](https://user-images.githubusercontent.com/23158313/149604028-915d9325-e52a-4378-ba58-17b7fe1a7a81.gif)

## Add, commit and push your changes

Please indicate in your commit's message the file that was modified using brakets. Por instance, if you modified the class DQ_Serialmanipulator, then you would do the following:
- `git commit -m "[DQ_SerialManipulator] your_message_explaining_the modification."`

![add_commit_push](https://user-images.githubusercontent.com/23158313/149603960-d69a8202-a3b1-4af5-a2d8-e1197cc26a81.gif)

## Open a draf pull request (More details [here](https://github.blog/2019-02-14-introducing-draft-pull-requests/))

Now, your draft pull request will be tested by Github actions automatically. 

![pull_request](https://user-images.githubusercontent.com/23158313/149604338-52f3ba35-ef25-440a-8bc8-75194c32130e.gif)

If your pull request fails the tests, don't worry, you will see where your code is not working. Pick your pull request in https://github.com/dqrobotics/matlab/pulls. Then, at the end of the page, click on 'Details'.

![failed_check](https://user-images.githubusercontent.com/23158313/149604965-677f783f-64af-4120-966a-0461c85f9418.gif)



