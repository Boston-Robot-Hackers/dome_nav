Run a full checkpoint in this order:

- Run all tests
- Run `ruff check . && ruff format --check .`; fix any violations before continuing
- Update current.md with your latest context
- Review changed code against judgment rules in @.claude/style_guide.md (design, error handling, naming semantics)
- Update any changed literate md files
- Create a commit (no need for a pull request)
- Push to github