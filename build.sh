clean=false
upload=false

while getopts "cu" opt; do
  case $opt in
    c)
      clean=true
      ;;
    u)
      upload=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      ;;
  esac
done

if $clean; then
  make clean
fi

time make -j$(($(sysctl -n hw.ncpu) + 1))
echo "compiled on $(date '+%Y-%m-%d %H:%M:%S')"

if $upload; then
  pros u
fi
