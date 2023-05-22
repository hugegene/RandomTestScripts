echo "Killing camera tabs"

{
     for KILLPID in `ps ax | grep read_videox | awk ' { print $1;}'`; do
      kill -9 $KILLPID;
     done
}||{
     echo "killed"
}
