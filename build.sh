reset=false;
clean=false
upload=false

while getopts "cur" opt; do
  case $opt in
    c)
      clean=true
      ;;
    u)
      upload=true
      ;;
    r)
      reset=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      ;;
  esac
done

if $reset; then
  reset
  source ~/.zshrc
fi

if $clean; then
  make clean
fi

time make -j$(($(sysctl -n hw.ncpu) + 1))
echo "compiled on $(date '+%Y-%m-%d %H:%M:%S')"

if $upload; then
  pros u
fi
