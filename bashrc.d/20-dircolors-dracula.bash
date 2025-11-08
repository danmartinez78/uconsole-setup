# Retro terminal LS_COLORS (green/amber/cyan on dark)
# Source from ~/.bashrc.d after installing this file.

command -v dircolors >/dev/null 2>&1 || return 0

# Inline dircolors database (classic terminal palette)
__DIRCOLORS='LS_COLORS=rs=0:di=1;32:ln=1;36:mh=00:pi=40;33:so=1;35:do=1;35:bd=40;33;01:cd=40;33;01:or=1;31:su=37;41:sg=30;43:tw=30;42:ow=34;42:st=37;44:ex=1;32:*.tar=1;33:*.tgz=1;33:*.zip=1;33:*.gz=1;33:*.xz=1;33:*.bz2=1;33:*.deb=1;33:*.rpm=1;33:*.jar=1;33:*.py=1;33:*.sh=1;33:*.bash=1;33:*.zsh=1;33:*.js=1;33:*.ts=1;33:*.rs=1;33:*.go=1;33:*.c=1;33:*.h=1;33:*.cpp=1;33:*.hpp=1;33:*.json=1;36:*.yml=36:*.yaml=36:*.toml=36:*.md=0;35:*.txt=0;37:'
export $(echo "$__DIRCOLORS")
unset __DIRCOLORS
