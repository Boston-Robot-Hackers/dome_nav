# Bootstrap Scaffold

When bootstrapping a new project, assume that you are inside the directory of the target. Check .claude folder is there and correct. Then create the following files and folders exactly as specified below.

## Folder structure to create

```
LICENSE
README.md
.gitignore
CLAUDE.md
01-literate/
02-doc/
  spec.md
  current.md
  notes.md
03-features/
  notdone/
  done/
  deferred/
  template.md
04-tasks/
  notdone/
  done/
  deferred/
  template.md
05-issues/
  template.md
```
### LICENSE
Copy from `.claude/templates/LICENSE.template` and replace `<YEAR>` and `<AUTHOR NAME>`.

### README.md
Copy from `.claude/templates/README.md.template` and replace `<APP NAME>` and other placeholders.

### .gitignore
Copy from `.claude/templates/.gitignore.template` as-is.

### CLAUDE.md
```
# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

Read and follow all rules in the `.claude/` folder:
- `.claude/process.md` — development workflow and feature/task tracking rules
- `.claude/codereview.md` — coding standards, style rules, and review checklist
- `02-doc/current.md` — session handoff and current status
- `02-doc/notes.md` — semi-permanent project notes

We are developing an app called <APP NAME>. Literate docs are in `01-literate/`,
project docs are in `02-doc/`, features are in `03-features/`, tasks are in
`04-tasks/`, issues are in `05-issues/`, and the spec is in `02-doc/spec.md`.
```

## After scaffolding

Prompt the user to:
1. Fill in `02-doc/spec.md` with the app description
2. Initialize `02-doc/current.md` as the session handoff file
3. Add any durable architecture notes to `02-doc/notes.md`
4. Replace `<APP NAME>` in `CLAUDE.md`, `README.md`, and `LICENSE` with the actual app name, author, and year
5. Define the first feature and matching task file before writing any code
