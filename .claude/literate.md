You are an expert in literate programming. Convert the following Python program 
into a literate programming document in Markdown format. Use mermaid for diagrams.

Structure the document as a narrative that a thoughtful programmer would want 
to read — not a comment-by-comment translation, but a coherent explanation of 
the program's design, intent, and logic.

Think of it as a true interactive lesson about algorithms, data structures and the way this code uses them.

Add a section at the end with observation on how the code can be improved in clarity, features, performance or any other ways.

Guidelines:
- Begin with a brief introduction explaining what the program does and why
- Organize sections around concepts and ideas, not necessarily the order 
  of the code
- Use prose to explain the *why* and *design decisions*, not just the *what*
- Embed code blocks inline within the narrative at the point where they 
  are discussed
- Each code block should be preceded by prose that motivates it
- Call out any non-obvious choices, tradeoffs, or assumptions

Output format: a single Markdown document with alternating prose and fenced Python code blocks. Include diagrams of flow, algorithm, interesting or tricky data structures. Do not include the full program text. Put the file in the `01-literate/` directory.

Filename: prefix with a two-digit number so files sort in logical reading sequence (e.g. `01-base_api.md`, `07-robot_controller.md`). Choose the number to reflect dependency order — foundational modules first, higher-level orchestration last. Number those that are sort of trivial or self evident as appendicixes as X01, X02 etc.

Frontmatter: begin every file with YAML frontmatter containing `version` (start at `1.0`) and `generated` (ISO 8601 date). Example:

```yaml
---
version: "1.0"
generated: "2026-05-04"
---
```
# If I ask you to create the final assembly literate pdf then do this:

1. Write a 00-overview.md which contains a "theory of operation" and a way for the reader to understand the package
2. Use pandoc to create a pdf from specifically all the numbered chapters not the appendices
3. Put them in numerical order and generate a command for example like this: 

pandoc file.md file2.md -o literate.pdf --pdf-engine=xelatex \
  -V monofont="Fira Code"

