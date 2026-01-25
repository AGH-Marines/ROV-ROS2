import os
import sys
import json
import subprocess
import re
from typing import List, Dict, Optional
import requests


class GitHubPRHandler:
    """Handle GitHub PR operations.

    This class provides methods to interact with the GitHub API for
    pull request operations, including fetching diffs, listing changed
    files, and posting review comments.

    :param str token: GitHub personal access token for API authentication
    :param str repo: Repository name in "owner/repo" format
    :param int pr_number: Pull request number
    """

    def __init__(self, token: str, repo: str, pr_number: int):
        """Initialize GitHub PR handler.

        :param str token: GitHub personal access token for API authentication
        :param str repo: Repository name in "owner/repo" format
        :param int pr_number: Pull request number
        """
        self.token = token
        self.repo = repo
        self.pr_number = pr_number
        self.api_url = f"https://api.github.com/repos/{repo}"
        self.headers = {
            "Authorization": f"token {token}",
            "Accept": "application/vnd.github.v3+json"
        }

    def get_pr_diff(self) -> str:
        """Get the PR diff.

        Fetches the complete git diff for the pull request from GitHub API.

        :return: The PR diff as a string
        :rtype: str
        :raises requests.HTTPError: If API request fails
        """
        url = f"{self.api_url}/pulls/{self.pr_number}.diff"
        response = requests.get(url, headers=self.headers)
        response.raise_for_status()
        return response.text

    def get_pr_files(self) -> List[Dict]:
        """Get all files changed in the PR.

        Fetches a list of all files that were changed, added,
        or deleted in the pull request.

        :return: List of file information dictionaries from GitHub API
        :rtype: list[dict]
        :raises requests.HTTPError: If API request fails
        """
        url = f"{self.api_url}/pulls/{self.pr_number}/files"
        response = requests.get(url, headers=self.headers)
        response.raise_for_status()
        return response.json()

    def create_review_comment(
        self,
        body: str,
        path: str,
        line: int,
        commit_id: str
    ) -> Dict:
        """Create a review comment on a specific line.

        Posts a comment on a specific line of code in the PR.

        :param str body: The comment text to post
        :param str path: File path relative to repository root
        :param int line: Line number to comment on
        :param str commit_id: SHA of the commit to comment on
        :return: Response data from GitHub API
        :rtype: dict
        :raises requests.HTTPError: If API request fails
        """
        url = f"{self.api_url}/pulls/{self.pr_number}/comments"
        data = {
            "body": body,
            "path": path,
            "line": line,
            "commit_id": commit_id
        }
        response = requests.post(url, headers=self.headers, json=data)
        response.raise_for_status()
        return response.json()

    def create_general_comment(self, body: str) -> Dict:
        """Create a general comment on the PR.

        Posts a general comment that appears in the PR conversation,
        not attached to any specific line of code.

        :param str body: The comment text to post
        :return: Response data from GitHub API
        :rtype: dict
        :raises requests.HTTPError: If API request fails
        """
        url = f"{self.api_url}/pulls/{self.pr_number}/comments"
        data = {"body": body}
        response = requests.post(url, headers=self.headers, json=data)
        response.raise_for_status()
        return response.json()

    def get_pr_head_commit(self) -> str:
        """Get the head commit SHA of the PR.

        Retrieves the SHA of the most recent commit in the PR,
        used for posting line-specific review comments.

        :return: The commit SHA as a string
        :rtype: str
        :raises requests.HTTPError: If API request fails
        """
        url = f"{self.api_url}/pulls/{self.pr_number}"
        response = requests.get(url, headers=self.headers)
        response.raise_for_status()
        return response.json()["head"]["sha"]


class CrushReviewer:
    """Handle Crush integration for code review.

    This class manages communication with Crush CLI tool,
    loading review templates, building prompts, and executing
    AI-powered code analysis.

    :param str project_path: Path to the project directory
    :param str template_name: Name of the review template to use
    :param config_path: Optional path to custom Crush configuration file
    :type config_path: str or None
    """

    def __init__(self, project_path: str, template_name: str = "ros2_review", config_path: str = None):
        """Initialize Crush reviewer.

        :param str project_path: Path to the project directory
        :param str template_name: Name of the review template to use (default: "ros2_review")
        :param config_path: Optional path to custom Crush configuration file
        :type config_path: str or None
        """
        self.project_path = project_path
        self.template_name = template_name
        self.config_path = config_path


    def analyze_code(self, diff: str, context: str = "") -> str:
        """Send code to Crush for analysis.

        Builds a review prompt from the provided diff and context,
        then executes the Crush CLI with the generated prompt.

        :param str diff: The PR diff to analyze
        :param str context: Additional context about the project
        :return: Crush's analysis as a string
        :rtype: str
        :raises subprocess.TimeoutExpired: If Crush analysis takes longer than timeout
        """
        # Build the prompt for Crush using templates
        prompt = self._build_review_prompt(diff, context)

        try:
            # Build Crush command
            crush_cmd = ["crush", "run", prompt]

            # Add custom config path if provided
            if self.config_path:
                # Create environment for Crush with custom config
                env = os.environ.copy()
                env["CRUSH_DATA_DIR"] = os.path.dirname(self.config_path)
            else:
                env = os.environ.copy()

            # Run Crush in non-interactive mode
            result = subprocess.run(
                crush_cmd,
                cwd=self.project_path,
                capture_output=True,
                text=True,
                timeout=300,  # 5 minute timeout
                env=env
            )

            if result.returncode != 0:
                print(f"Error running Crush: {result.stderr}", file=sys.stderr)
                return f"Error: Could not run Crush analysis. {result.stderr}"

            return result.stdout

        except subprocess.TimeoutExpired:
            return "Error: Crush analysis timed out."
        except Exception as e:
            return f"Error running Crush: {str(e)}"

    def _load_template(self, template_name: str) -> str:
        """Load a prompt template from the templates directory.

        Attempts to load a markdown template file from the
        .github/templates directory. Returns None if template
        is not found.

        :param str template_name: Name of the template (without .md extension)
        :return: Template content as a string, or None if not found
        :rtype: str or None
        """
        template_path = os.path.join(
            self.project_path,
            ".github/templates",
            f"{template_name}.md"
        )

        if os.path.exists(template_path):
            with open(template_path, "r") as f:
                return f.read()

        # Fallback to built-in template if custom template doesn't exist
        return None

    def _build_review_prompt(self, diff: str, context: str) -> str:
        """Build the prompt for Crush code review using templates.

        Loads the specified review template and formats it with
        the provided diff and context. Falls back to a built-in
        template if a custom template is not found.

        :param str diff: The PR diff to include in the prompt
        :param str context: Project context to include in the prompt
        :return: The formatted review prompt
        :rtype: str
        """
        # Try to load custom template
        template = self._load_template(self.template_name)

        if template:
            # Use custom template
            return template.format(diff=diff, context=context)
        else:
            # Fallback to built-in template
            return f"""You are a code reviewer for a ROS2 project. Review the following PR changes.

{context}

PR DIFF:
{diff}

Please provide:
1. **Overall Assessment**: Brief summary of the changes
2. **Specific Issues**: Line-by-line issues with suggestions (format as "FILE:LINE: Issue - Suggestion")
3. **Best Practices**: Any violations of ROS2 or Python/C++ best practices
4. **Security Concerns**: Any potential security issues
5. **Positive Feedback**: What looks good

Format your response clearly with sections. For line-specific issues, use the format:
```
FILE:LINE: [severity] Issue description
Suggested fix: ...
```

Be concise and actionable. Focus on important issues, not nitpicking.
"""


class ReviewCommentParser:
    """Parse Crush's output into structured review comments.

    This class provides static methods to parse the natural language
    output from Crush AI and extract structured review data including
    line-specific comments, best practice suggestions, security issues,
    and positive feedback.
    """

    @staticmethod
    def parse_crush_output(output: str) -> Dict:
        """Parse Crush's output into structured data.

        Parses the natural language output from Crush AI and extracts
        structured data organized by review categories.

        :param str output: The raw output text from Crush AI
        :return: Dictionary with the following keys:

            - overall (str): Overall assessment text
            - line_comments (list[dict]): List of line-specific comments
              with 'path', 'line', and 'body' keys
            - best_practices (list[str]): Best practice suggestions
            - security (list[str]): Security concerns
            - positive (list[str]): Positive feedback
        :rtype: dict
        """
        result = {
            "overall": "",
            "line_comments": [],
            "best_practices": [],
            "security": [],
            "positive": []
        }

        current_section = None
        lines = output.split("\n")

        for line in lines:
            # Parse file:line comments
            match = re.match(r"^([^\s:]+):(\d+):\s*(.+)", line)
            if match:
                path = match.group(1)
                line_num = int(match.group(2))
                comment = match.group(3)
                result["line_comments"].append({
                    "path": path,
                    "line": line_num,
                    "body": comment
                })
                continue

            # Parse sections
            line_lower = line.lower()
            if "overall assessment" in line_lower or "overall" in line_lower:
                current_section = "overall"
                continue
            elif "specific issues" in line_lower or "line-specific" in line_lower:
                current_section = "line_comments"
                continue
            elif "best practices" in line_lower:
                current_section = "best_practices"
                continue
            elif "security" in line_lower:
                current_section = "security"
                continue
            elif "positive" in line_lower or "good" in line_lower:
                current_section = "positive"
                continue

            # Add content to current section
            if current_section and line.strip():
                if current_section == "overall":
                    result["overall"] += line + "\n"
                elif current_section == "best_practices" and line.strip().startswith("-"):
                    result["best_practices"].append(line.strip("- ").strip())
                elif current_section == "security" and line.strip().startswith("-"):
                    result["security"].append(line.strip("- ").strip())
                elif current_section == "positive" and line.strip().startswith("-"):
                    result["positive"].append(line.strip("- ").strip())

        return result


def main():
    """Main entry point for the code review bot.

    Orchestrates the complete code review workflow:
    1. Validates required environment variables
    2. Initializes GitHub and Crush handlers
    3. Loads project context from AGENTS.md
    4. Fetches PR diff from GitHub
    5. Sends diff to Crush for AI analysis
    6. Parses Crush output into structured data
    7. Posts review comments to the PR

    Required environment variables:
        - GITHUB_TOKEN: GitHub API authentication token
        - GITHUB_REPOSITORY: Repository name (owner/repo)
        - PR_NUMBER: Pull request number

    Optional environment variables:
        - PROJECT_PATH: Path to the project directory (default: current working directory)
        - CRUSH_TEMPLATE: Name of the review template to use (default: \"ros2_review\")
        - CRUSH_CONFIG_PATH: Path to custom Crush configuration file

    :return: None (exits with status code 0 on success, 1 on error)
    :rtype: None
    """
    # Get environment variables
    github_token = os.getenv("GITHUB_TOKEN")
    if not github_token:
        print("Error: GITHUB_TOKEN environment variable not set", file=sys.stderr)
        sys.exit(1)

    github_repo = os.getenv("GITHUB_REPOSITORY")
    if not github_repo:
        print("Error: GITHUB_REPOSITORY environment variable not set", file=sys.stderr)
        sys.exit(1)

    pr_number = os.getenv("PR_NUMBER")
    if not pr_number:
        print("Error: PR_NUMBER environment variable not set", file=sys.stderr)
        sys.exit(1)
    pr_number = int(pr_number)

    project_path = os.getenv("PROJECT_PATH", os.getcwd())

    # Get optional template name
    template_name = os.getenv("CRUSH_TEMPLATE", "ros2_review")
    print(f"Using template: {template_name}")

    # Get optional Crush config path
    crush_config_path = os.getenv("CRUSH_CONFIG_PATH")
    if crush_config_path:
        print(f"Using Crush config: {crush_config_path}")

    # Initialize handlers
    print(f"Starting review for PR #{pr_number} in {github_repo}...")
    pr_handler = GitHubPRHandler(github_token, github_repo, pr_number)
    crush_reviewer = CrushReviewer(
        project_path,
        template_name=template_name,
        config_path=crush_config_path
    )

    # Load project context
    context = ""
    agents_md = os.path.join(project_path, "AGENTS.md")
    if os.path.exists(agents_md):
        with open(agents_md, "r") as f:
            # Read first 100 lines to avoid overwhelming Crush
            context_lines = [f.readline() for _ in range(100)]
            context = "\n".join(context_lines)
            context = "\nPROJECT CONTEXT:\n" + context

    # Get PR diff
    print("Fetching PR diff...")
    try:
        diff = pr_handler.get_pr_diff()
        print(f"Diff size: {len(diff)} characters")
    except Exception as e:
        print(f"Error fetching PR diff: {e}", file=sys.stderr)
        sys.exit(1)

    # Send to Crush for analysis
    print("Analyzing with Crush...")
    crush_output = crush_reviewer.analyze_code(diff, context)
    print("Crush analysis complete.")
    print(f"\n{crush_output}\n")

    # Parse Crush output
    print("Parsing review suggestions...")
    parsed_review = ReviewCommentParser.parse_crush_output(crush_output)

    # Post general comment with overall assessment
    if parsed_review["overall"]:
        general_comment = f"""## 🤖 Crush Code Review

**Overall Assessment:**
{parsed_review["overall"]}
"""
        if parsed_review["best_practices"]:
            general_comment += "\n**Best Practice Suggestions:**\n"
            for bp in parsed_review["best_practices"]:
                general_comment += f"- {bp}\n"

        if parsed_review["security"]:
            general_comment += "\n**Security Concerns:**\n"
            for sec in parsed_review["security"]:
                general_comment += f"- {sec}\n"

        if parsed_review["positive"]:
            general_comment += "\n**Positive Feedback:**\n"
            for pos in parsed_review["positive"]:
                general_comment += f"- {pos}\n"

        try:
            pr_handler.create_general_comment(general_comment)
            print("Posted general review comment.")
        except Exception as e:
            print(f"Error posting general comment: {e}", file=sys.stderr)

    # Post line-specific comments
    if parsed_review["line_comments"]:
        print(f"Posting {len(parsed_review['line_comments'])} line-specific comments...")
        commit_id = pr_handler.get_pr_head_commit()

        for comment in parsed_review["line_comments"]:
            try:
                pr_handler.create_review_comment(
                    body=comment["body"],
                    path=comment["path"],
                    line=comment["line"],
                    commit_id=commit_id
                )
                print(f"  -> Posted comment on {comment['path']}:{comment['line']}")
            except Exception as e:
                print(f"  -> Error posting comment on {comment['path']}:{comment['line']}: {e}")
                continue

    print("Code review complete!")


if __name__ == "__main__":
    main()
