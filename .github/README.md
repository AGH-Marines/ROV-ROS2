# Crush Code Review Bot

Automated code review for GitHub pull requests powered by Crush AI and GLM-4.7 model.

## Overview

The bot provides intelligent, context-aware code reviews on every PR:

- 🤖 **AI-Powered**: Uses GLM-4.7 model for code analysis
- 📋 **Structured Feedback**: Categorized issues (Critical/Important/Suggestions)
- 🎯 **Context-Aware**: Leverages project documentation (AGENTS.md)
- ⚡ **Fast**: Reviews complete in 1-3 minutes
- 🔒 **Secure**: API keys stored in GitHub Secrets, not in code

## Directory Structure

```
.github/
├── README.md                    # This file - complete documentation
├── workflows/
│   └── code-review-bot.yaml     # GitHub Actions workflow definition
├── scripts/
│   ├── code_review_bot.py       # Main bot script (Python)
│   └── requirements.txt        # Python dependencies
└── templates/
    ├── ros2_review.md          # ROS2-specific review template
    └── general_review.md       # General-purpose review template
```

## Quick Start (5 minutes)

### 1. Get API Key

Visit [Z.AI Platform](https://open.bigmodel.cn/) and create an API key.

### 2. Add to GitHub Secrets

1. Go to your repository → **Settings** → **Secrets and variables** → **Actions**
2. Click **New repository secret**
3. **Name**: `ZAI_API_KEY`
4. **Secret**: Paste your Z.AI API key
5. Click **Add secret**

## File Descriptions

### Workflow: `workflows/code-review-bot.yaml`

**Purpose**: GitHub Actions workflow that triggers on PR events and runs the bot.

**Triggers**:
- Pull request opened
- Pull request updated (new commits)
- Pull request reopened
- Manual trigger (via Actions tab)

**Key Steps**:
1. **Checkout code**: Gets your repository
2. **Set up Python**: Installs Python 3.10
3. **Install dependencies**: Downloads required Python packages
4. **Install Crush CLI**: Downloads and installs Crush
5. **Configure Crush**: Generates runtime configuration for GLM-4.7
6. **Run Code Review Bot**: Executes the bot script
7. **Post Review Summary**: Adds status comment to PR

**Configuration**:
- Uses GLM-4.7 model from Z.AI
- Reads API key from `ZAI_API_KEY` GitHub Secret
- 5-minute timeout for Crush analysis
- `continue-on-error: true` (won't block PR if bot fails)

**Modifying**:
- Change `CRUSH_TEMPLATE` to use different review template
- Adjust `timeout-minutes` for large PRs
- Add conditional steps (e.g., filter by file types)

---

### Bot Script: `scripts/code_review_bot.py`

**Purpose**: Python script that integrates Crush AI with GitHub API.

**Key Components**:

1. **GitHubPRHandler Class**
   - Fetches PR diffs
   - Gets list of changed files
   - Posts review comments (line-specific)
   - Posts general summary comments

2. **CrushReviewer Class**
   - Loads review templates
   - Builds prompts with project context
   - Executes Crush CLI
   - Returns structured analysis

3. **ReviewCommentParser Class**
   - Parses Crush's natural language output
   - Extracts line-specific comments
   - Categorizes issues by severity

4. **main() Function**
   - Reads environment variables
   - Orchestrates the review process
   - Posts comments to PR

**Environment Variables**:
- `GITHUB_TOKEN`: GitHub API token (auto-provided)
- `GITHUB_REPOSITORY`: Repository name (auto-provided)
- `PR_NUMBER`: PR number to review (auto-provided)
- `PROJECT_PATH`: Path to codebase (auto-provided)
- `CRUSH_CONFIG_PATH`: Path to Crush config (auto-provided)
- `CRUSH_TEMPLATE`: Template name to use (default: `ros2_review`)
- `ZAI_API_KEY`: Z.AI API key (from GitHub Secret)

**Modifying**:
- Adjust `analyze_code()` timeout (default: 300 seconds)
- Change comment parsing logic in `ReviewCommentParser`
- Add custom filters in `get_pr_files()`
- Modify prompt building in `_build_review_prompt()`

---

### Dependencies: `scripts/requirements.txt`

**Purpose**: Python packages required by the bot.

**Contents**:
```
requests>=2.31.0
```

**Installing Locally**:
```bash
pip install -r .github/scripts/requirements.txt
```

---

### Template: `templates/ros2_review.md`

**Purpose**: ROS2-specific code review template.

**Focus Areas**:
- Python (rclpy) patterns
- C++ (rclcpp) patterns
- Launch file best practices
- tf2 transforms
- Parameter handling
- ROS2 conventions

**When Used**: Default template for ROS2 projects.

**Modifying**:
- Add project-specific ROS2 patterns
- Adjust severity definitions
- Add custom review criteria
- Update coding conventions

---

### Template: `templates/general_review.md`

**Purpose**: General-purpose code review template.

**Focus Areas**:
- Code quality
- Best practices
- Security
- Performance
- Maintainability

**When Used**: Set `CRUSH_TEMPLATE: "general_review"` in workflow.

**Modifying**:
- Customize for different languages/frameworks
- Add project-specific rules
- Adjust output format

---

## Configuration Guide

### GLM-4.7 Model (Recommended)

The bot is pre-configured to use GLM-4.7 from Z.AI:

**Model Details**:
- **Model ID**: `glm-4`
- **Context Window**: 128,000 tokens (handles large PRs)
- **Max Output**: 4,096 tokens (detailed reviews)
- **API Endpoint**: https://open.bigmodel.cn/api/paas/v4/

**How It Works**:
1. Workflow generates temporary `crush.json` at runtime
2. Reads `ZAI_API_KEY` from GitHub Secrets
3. Configures Crush to use Z.AI provider
4. Analyzes code with GLM-4.7
5. Temporary config is deleted after workflow completes

**Benefits**:
- ✅ No repository modifications needed
- ✅ API key stored securely
- ✅ Large context window for big PRs
- ✅ High-quality analysis

---

### Alternative Configurations

#### Use Different Template

Edit `.github/workflows/code-review-bot.yaml`:

```yaml
env:
  CRUSH_TEMPLATE: "general_review"  # Instead of ros2_review
```

#### Use Custom Template

1. Create `.github/templates/my_template.md`:
```markdown
You are a code reviewer...

PR Changes:
{diff}

[Your custom review format]
```

2. Update workflow:
```yaml
env:
  CRUSH_TEMPLATE: "my_template"
```

#### Use Different Provider

Change the "Configure Crush" step in workflow:

**OpenAI**:
```yaml
- name: Configure Crush with OpenAI
  env:
    OPENAI_API_KEY: ${{ secrets.OPENAI_API_KEY }}
  run: |
    cat > $HOME/.config/crush/crush.json << 'EOF'
    {
      "providers": {
        "openai": {
          "name": "OpenAI",
          "type": "openai",
          "api_key_env": "OPENAI_API_KEY",
          "models": [
            {
              "id": "gpt-4",
              "context_window": 8192,
              "default_max_tokens": 4096
            }
          ]
        }
      }
    }
    EOF
```

Add `OPENAI_API_KEY` to GitHub Secrets.

#### Filter Files to Review

Modify `scripts/code_review_bot.py`:

```python
def get_pr_files(self) -> List[Dict]:
    url = f"{self.api_url}/pulls/{self.pr_number}/files"
    response = requests.get(url, headers=self.headers)
    response.raise_for_status()
    all_files = response.json()

    # Only review Python files
    return [f for f in all_files if f['filename'].endswith('.py')]
```

#### Adjust PR Size Limits

Add check in workflow before running bot:

```yaml
- name: Check PR Size
  id: check_size
  run: |
    LINES=$(git diff --stat HEAD~1 HEAD | awk '{sum+=$3} END {print sum+0}')
    if [ "$LINES" -gt 2000 ]; then
      echo "PR too large, skipping review"
      echo "skip=true" >> $GITHUB_OUTPUT
    else
      echo "skip=false" >> $GITHUB_OUTPUT
    fi

- name: Run Code Review Bot
  if: steps.check_size.outputs.skip != 'true'
  run: |
    python .github/scripts/code_review_bot.py
```

---

## How It Works

```
1. PR Created/Updated
        ↓
2. GitHub Action Triggers
        ↓
3. Workflow Reads ZAI_API_KEY (Secret)
        ↓
4. Generates Temporary crush.json
        ↓
5. Bot Fetches PR Diff
        ↓
6. Bot Loads AGENTS.md Context
        ↓
7. Bot Builds Prompt from Template
        ↓
8. Crush Analyzes with GLM-4.7
        ↓
9. Bot Parses Analysis
        ↓
10. Bot Posts Comments:
    - General summary
    - Line-specific issues
    - Status update
```

---

## Review Output

### Summary Comment

Posted at top of PR:
```markdown
## 🤖 Crush Code Review Bot Status

**Status**: success

✅ Code review completed successfully!

Check the comments above for detailed suggestions from Crush.
```

### Line-Specific Comments

Posted on relevant code lines:
```
File: src/my_package/nodes/my_node.py:42

🟡 Missing error handling for potential None return

The parameter value could be None if not properly declared.
Consider adding validation:
```python
value = self.get_parameter('param').value
if value is None:
    self.get_logger().error('Parameter not set')
    return
```
```

### Feedback Categories

- 🔴 **Critical Issues**: Must fix before merging
- 🟡 **Important Issues**: Should fix (significant problems)
- 🔵 **Suggestions**: Nice to have improvements
- ✅ **Positive Feedback**: What looks good

---

## Project Context

The bot uses `AGENTS.md` in your project root to provide better reviews:

**What's Included**:
- Project overview
- Code conventions
- Architecture patterns
- Common pitfalls
- Technical details

**Best Practices for AGENTS.md**:
- Keep it comprehensive
- Include coding standards
- Document common patterns
- Add architecture notes
- Update regularly

**Example Sections**:
```markdown
## Project Overview
Brief description of what the project does

## Code Style
- Line length limits
- Naming conventions
- File organization

## Common Patterns
- Node lifecycle patterns
- Parameter handling
- Error handling

## Known Issues
- Common mistakes to avoid
- Anti-patterns
```

---

## Troubleshooting

### Bot Doesn't Run

**Checklist**:
- [ ] Workflow file exists in `.github/workflows/code-review-bot.yaml`
- [ ] GitHub Actions is enabled in repository
- [ ] Workflow has `pull-requests: write` permission
- [ ] Check Actions tab for error messages

**Solution**: Verify workflow YAML syntax and permissions.

---

### "ZAI_API_KEY Not Found"

**Cause**: Secret not added or name is wrong.

**Solution**:
1. Go to Repository → Settings → Secrets → Actions
2. Verify secret named `ZAI_API_KEY` (case-sensitive, no spaces)
3. Ensure secret value is not empty
4. Re-add secret if needed

---

### "Invalid API Key"

**Cause**: Wrong, expired, or invalid API key.

**Solution**:
1. Verify API key is correct (copy again from Z.AI)
2. Check API key hasn't expired on Z.AI dashboard
3. Ensure account has API access
4. Test key locally:
   ```bash
   curl https://open.bigmodel.cn/api/paas/v4/models \
     -H "Authorization: Bearer YOUR_API_KEY"
   ```

---

### No Comments on PR

**Cause**: Permission issue or API error.

**Checklist**:
- [ ] `pull-requests: write` permission is set in workflow
- [ ] Workflow completed successfully (green checkmark)
- [ ] Check workflow logs for API errors
- [ ] PR number is correct
- [ ] Bot script didn't fail silently

**Solution**: Review workflow logs in Actions tab, look for API errors.

---

### Generic Reviews (Not ROS2-Specific)

**Cause**: Context not loaded or wrong template.

**Checklist**:
- [ ] `AGENTS.md` exists in project root
- [ ] `AGENTS.md` has content
- [ ] Template is set to `ros2_review`
- [ ] Context is being loaded in bot script

**Solution**: Verify `AGENTS.md` exists and template is correct.

---

### Workflow Times Out

**Cause**: PR too large or API slow.

**Solutions**:
1. **Split PR**: Create smaller PRs (< 2000 lines)
2. **Increase timeout**: Edit workflow:
   ```yaml
   timeout-minutes: 10  # Increase from default
   ```
3. **Filter files**: Only review relevant file types
4. **Use faster model**: GLM-4-Flash (if available)

---

### Reviews Are Too Long/Verbose

**Cause**: Template asks for too much detail.

**Solution**: Edit template to be more concise:
```markdown
Provide concise feedback:
- 3-5 critical issues max
- 5-10 important issues max
- Keep suggestions brief
- Skip minor nitpicks
```

---

## Testing Locally

### Prerequisites

```bash
# Install dependencies
pip install -r .github/scripts/requirements.txt

# Install Crush
curl -fsSL https://raw.githubusercontent.com/charmbracelet/crush/main/install.sh | sh
```

### Test Crush

```bash
# Basic test
crush run "Say hello"

# Code review test
crush run "Review this:\ndef foo():\n    pass"
```

### Test Bot Script

```bash
export ZAI_API_KEY="your-api-key"
export GITHUB_TOKEN="your-personal-access-token"
export GITHUB_REPOSITORY="org/repo"
export PR_NUMBER=123
export PROJECT_PATH=$(pwd)
export CRUSH_TEMPLATE="ros2_review"

python .github/scripts/code_review_bot.py
```

---

## Cost and Performance

### GLM-4.7 Pricing

*Check [Z.AI Pricing](https://open.bigmodel.cn/) for current rates.*

**Estimated Cost per Review**:
- Small PR (< 100 lines): ~$0.001
- Medium PR (100-500 lines): ~$0.005
- Large PR (500-2000 lines): ~$0.02
- Very Large PR (2000+ lines): ~$0.05+

**Monthly Estimates**:
- Small team (20 PRs): ~$0.50/month
- Medium team (50 PRs): ~$1.00/month
- Large team (100 PRs): ~$2.00/month

### Performance

- **Small PR**: ~30 seconds
- **Medium PR**: ~1-2 minutes
- **Large PR**: ~3-5 minutes
- **Very Large PR**: May timeout (split recommended)

---

## Security

### What's Secure

✅ **API Keys**: Stored in GitHub Secrets, never in code
✅ **Temporary Config**: Generated at runtime, deleted after use
✅ **No Repo Changes**: Your `crush.json` is never modified
✅ **Encrypted**: Secrets encrypted at rest and in transit
✅ **Access Control**: Only Actions can access secrets

### Best Practices

1. **Never commit API keys** to repository
2. **Rotate API keys** every 90 days
3. **Monitor API usage** on Z.AI dashboard
4. **Review workflow logs** for accidental secret exposure
5. **Limit secret scope** to Actions only
6. **Use repository tokens** instead of personal tokens

---

## Advanced Usage

### Conditional Reviews

Only review PRs with specific label:

```yaml
on:
  pull_request:
    types: [opened, synchronize, reopened, labeled]

jobs:
  code-review:
    if: contains(github.event.pull_request.labels.*.name, 'needs-review')
    runs-on: ubuntu-latest
    # ... rest of workflow
```

### Multiple Workflows

Run different reviews for different packages:

```yaml
jobs:
  review-ros2:
    if: contains(github.event.pull_request.changed_files, 'src/ros2')
    run: python .github/scripts/code_review_bot.py
    env:
      CRUSH_TEMPLATE: "ros2_review"

  review-python:
    if: contains(github.event.pull_request.changed_files, '.py')
    run: python .github/scripts/code_review_bot.py
    env:
      CRUSH_TEMPLATE: "python_review"
```

### Integration with Tests

Run review only after tests pass:

```yaml
jobs:
  test:
    # ... test job

  code-review:
    needs: test
    if: needs.test.result == 'success'
    # ... review job
```

### Custom Severity Labels

Modify template to use custom severity:

```markdown
## 🛑 Blocker (must fix)
## 🔴 Critical (should fix)
## 🟠 Major (good to fix)
## 🟡 Minor (optional)
```

---

## Support and Resources

### Documentation

- **This File**: Complete documentation
- [Z.AI API Docs](https://open.bigmodel.cn/dev/api)
- [Crush Documentation](https://charm.sh/crush)
- [GitHub Actions Docs](https://docs.github.com/en/actions)

### Getting Help

1. **Check workflow logs**: Actions tab → Latest run → Expand steps
2. **Verify configuration**: Check Secrets and workflow YAML
3. **Test locally**: Use local testing instructions above
4. **Review examples**: Check template files for patterns

### Common Issues

| Issue | Solution |
|-------|----------|
| Secret not found | Add `ZAI_API_KEY` to GitHub Secrets |
| Invalid API key | Verify key in Z.AI dashboard |
| No comments | Check `pull-requests: write` permission |
| Timeout | Split large PRs or increase timeout |
| Generic reviews | Ensure `AGENTS.md` exists |

---

## Changelog

### Current Version

- ✅ GLM-4.7 model integration
- ✅ GitHub Secrets for API key
- ✅ Runtime Crush configuration
- ✅ ROS2-specific templates
- ✅ Line-specific comments
- ✅ Structured feedback categories

### Planned Enhancements

- [ ] Draft reviews (manual approval)
- [ ] File type filtering
- [ ] PR size limits
- [ ] Review scoring
- [ ] Trend analysis
- [ ] Auto-fix suggestions

---

## License

This bot is provided as-is for use in your projects.

## Credits

Built with:
- [Crush AI](https://charm.sh/crush/) - AI code analysis
- [GLM-4.7](https://open.bigmodel.cn/) - Large language model
- GitHub Actions - Automation
- Python requests - GitHub API interaction
