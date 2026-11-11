# GitHub Actions setup

Local verification of the public snapshot passed: 16 tests, 77 asset hashes, 66 task records, six rack nominal transfers and three open-hand exits.

The workflow is provided as [github-actions-validate.yml](github-actions-validate.yml). It is **not active**. During the initial publication GitHub rejected a push containing `.github/workflows/validate.yml` because the publisher's existing OAuth credential lacked the `workflow` scope. No workflow was enabled with that credential.

After using a GitHub credential with permission to manage workflows, install the template at `.github/workflows/validate.yml` and push it. The workflow uses read-only repository permissions, installs the pinned Python dependencies, checks recorded asset/task evidence, runs the tests and rechecks the six rack tasks and three nominal open-hand exits.

The same checks are already runnable locally through the commands in the repository README; enabling Actions is not required to use the benchmark.
