# Welcome, contributors to DQ Robotics!

We are happy about your interest in our project! DQ Robotics is a standalone, **fully moderated** open-source library, and contributions are always welcome! 

If your proposed modifications change current functionality, you **must** first discuss them with the developers team and ensure they are approved. Otherwise, it will be rejected without review.

This is a set of guidelines for contributing to [DQ Robotics](https://dqrobotics.github.io/).

# Workflow

- Fork the [master branch of dqrobotics/matlab](https://github.com/dqrobotics/matlab).
- Propose your modifications and open a draft pull request. Keep in mind the following **requirements**:
   - Propose individual changes (several changes of the same type are allowed on the same pull request).
   - Do not unnecessarily change any internal implementation that is working correctly without prior approval.
   - Include a clear and concise rationale behind each pull request.
- Your modifications will be tested automatically through Github actions. Github actions run the tests of [matlab-tests](https://github.com/dqrobotics/matlab-tests), which execute all the examples of [matlab-examples](https://github.com/dqrobotics/matlab-examples). 
- Once your draft pull request passes all the tests, you can switch the status to pull request. (More details [here](https://github.blog/2019-02-14-introducing-draft-pull-requests/)).

<img src="https://user-images.githubusercontent.com/23158313/206892365-388afa7f-588b-45d2-b7c3-fd01966e6b29.png" alt="drawing" width="1200"/>

## Case 1 (Common cases)

- If your modifications pass all tests, a designated member of our team will review all the changes and they will accept them after all necessary adjustments, if any. 

## Case 2 (Very rare cases)

- In some cases, your modifications would fail some tests because of incompatibility with the current version of [matlab-tests](https://github.com/dqrobotics/matlab-tests) and/or [matlab-examples](https://github.com/dqrobotics/matlab-examples). In thoses cases, you must propose changes in [matlab-tests](https://github.com/dqrobotics/matlab-tests) and [matlab-examples](https://github.com/dqrobotics/matlab-examples) to make them compatible with your new version of the dqrobotics/matlab. 
- A designated member of our team will review all the changes proposed in both [matlab-tests](https://github.com/dqrobotics/matlab-tests) and [matlab-examples](https://github.com/dqrobotics/matlab-examples). They will accept the modifications in the master branch after all necessary adjustments. At this point, it is expected that your pull request passes all the tests in the master branch but fails in the release branch.

![master_and_release](https://user-images.githubusercontent.com/23158313/150379489-cabc85bb-dbe4-41be-a405-7b254a36092a.png)

- After your dqrobotics/matlab pull request is adjusted accordingly and passes all the tests, your proposed modifications will be scrutinized to ensure they follow the coding style and development philosophy, are technically correct, and add value to the current implementation. 
- Finally, your modifications will be accepted in the master branch. The approver will squash and merge to ensure that there is just one commit file.

# Example

## Fork the [dqrobotics/matlab](https://github.com/dqrobotics/matlab) in your Github account

![fork_master](https://user-images.githubusercontent.com/23158313/149602838-133f6c09-2e16-418e-8ab6-47fb36a91056.gif)

## Clone the forked repository

For instance, if your forked matlab respository is https://github.com/juanjqo/matlab, then

Type in your terminal:

- `git clone https://github.com/juanjqo/matlab.git`

![git_clone](https://user-images.githubusercontent.com/23158313/149603381-78732b55-2794-4be9-9a12-b7062d0649b5.gif)

## Make your modifications

Now, you can modify the code with your contributions.
(In this specific example, as shown in the animated figure below, the CONTRIBUTING.md file is modified.)

![modifications](https://user-images.githubusercontent.com/23158313/149604028-915d9325-e52a-4378-ba58-17b7fe1a7a81.gif)

## Add, commit and push your changes

Please indicate in your commit message, using brackets, the modified file. For instance, if you modified the class `DQ_SerialManipulator`, then you would do the following:

- `git commit -m "[DQ_SerialManipulator] your_message_explaining_the modification."`

![add_commit_push](https://user-images.githubusercontent.com/23158313/149603960-d69a8202-a3b1-4af5-a2d8-e1197cc26a81.gif)

However, if your explanation is longer or affects more than one file, you must write a more meaninful commit using a text editor, and hence **would not** use the `-m` option.

For example, this is how a longer commit would be written:
```
1: This commit solves issue X affecting `DQ_CLASS_A` and all its subclasses.
2:
3: [DQ_CLASS_A] Deleted variable X and included method Y to ensure correctness of model Z.
4: More specifically, Z was returning the transformation from frame A to frame B, instead 
5: of frame B to frame A, as described in Eq. (X) of paper [full reference here]. This has 
6: been fixed. 
7:
8: [DQ_CLASS_A_SUBCLASS_C] Changed method Z to comply with the modification made in 
9: DQ_CLASS_A

```







## Open a draft pull request (More details [here](https://github.blog/2019-02-14-introducing-draft-pull-requests/))

Now, your draft pull request will be tested by Github actions automatically. 

![pull_request](https://user-images.githubusercontent.com/23158313/149604338-52f3ba35-ef25-440a-8bc8-75194c32130e.gif)

If your pull request fails the tests, don't worry!, you will see where your code is not working. Pick your pull request in https://github.com/dqrobotics/matlab/pulls. Then, at the end of the page, click on 'Details'.

![failed_check](https://user-images.githubusercontent.com/23158313/149604965-677f783f-64af-4120-966a-0461c85f9418.gif)
