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
  history.md
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
  chores.md
  template.md
05-issues/
  open/
  closed/
  deferred/
  template.md
run.bash
```
### LICENSE
Copy from `.claude/templates/LICENSE.template` and replace `<YEAR>` and `<AUTHOR NAME>`.

### README.md
Copy from `.claude/templates/README.md.template` and replace `<APP NAME>` and other placeholders.

### .gitignore
Copy from `.claude/templates/.gitignore.template` as-is.

### CLAUDE.md
Copy from `.claude/templates/CLAUDE.md.template` and replace `<APP NAME>`.

### run.bash
Executable shell script containing the app's run command; set executable with `chmod +x run.bash`.

## After scaffolding

Prompt the user to:
1. Fill in `02-doc/spec.md` with the app description
2. Initialize `02-doc/current.md` as the session handoff file — keep it to just
   an `## Open` section (what's in progress/next); when work is marked done,
   move that entry out of `current.md` into `02-doc/history.md` rather than
   letting it accumulate in the always-read file
3. Create `02-doc/history.md` with a one-line header (e.g. "# History") — it
   starts empty and only grows as work is marked done
4. Add any durable architecture notes to `02-doc/notes.md`
5. Replace `<APP NAME>` in `CLAUDE.md`, `README.md`, and `LICENSE` with the actual app name, author, and year
6. Fill in `settings.json`'s `autoMode.environment` block with the real project
   purpose, package manager, run/test commands, and source control location
7. Define the first feature and matching task file before writing any code
