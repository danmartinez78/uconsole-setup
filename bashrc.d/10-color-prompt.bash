# Enable a vivid, Dracula-flavored prompt with git + venv hints
# Safe to source from ~/.bashrc.d on Ubuntu

# Only for interactive shells
[ -z "$PS1" ] && return

# Basic color helpers (256-color)
esc="\[\e"
end="m\]"
RESET="${esc}[0${end}"
FG() { printf "%s[38;5;%s%s" "$esc" "$1" "$end"; }

# Retro terminal palette (green on black with amber accents)
C_ACC=$(FG 214)   # amber accent
C_CYN=$(FG  51)   # bright cyan
C_GRN=$(FG  46)   # terminal green
C_YLW=$(FG 214)   # amber/yellow
C_PNK=$(FG 214)   # use amber instead
C_RED=$(FG 196)   # bright red (errors only)
C_DIM=$(FG 240)   # dim gray

# Git segment: prints " on <branch>" and a dirty marker if needed
__git_segment() {
  command -v git >/dev/null 2>&1 || return 0
  local b dirty
  b=$(git symbolic-ref --short -q HEAD 2>/dev/null || git describe --tags --always 2>/dev/null) || return 0
  if ! git diff --no-ext-diff --quiet 2>/dev/null; then dirty="*"; fi
  printf " %s %s%s" "$C_PNK" "$b" "$dirty"
}

# Python venv name (if any)
__venv_segment() {
  [ -n "$VIRTUAL_ENV" ] || return 0
  printf " %s(%s)" "$C_YLW" "$(basename "$VIRTUAL_ENV")"
}

# Exit status indicator (only if non-zero)
__rc_segment() {
  local rc=$?
  [ $rc -eq 0 ] && return 0
  printf " ${C_RED}✘${C_DIM}%d" "$rc"
}

# Build prompt with dynamic segments
__build_prompt() {
  # Set terminal title to user@host:path
  printf '\033]0;%s@%s:%s\007' "$USER" "${HOSTNAME%%.*}" "${PWD/#$HOME/~}"
  
  PS1="${C_CYN}\A ${C_ACC}\u@\h ${C_GRN}\w${RESET}$(__git_segment)$(__venv_segment)$(__rc_segment)\n$ ${RESET}"
}

PROMPT_COMMAND=__build_prompt

# quality-of-life colorized tools
alias ls='ls --color=auto'
alias grep='grep --color=auto'
