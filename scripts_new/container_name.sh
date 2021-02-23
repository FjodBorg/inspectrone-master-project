DIR="$(cd "$(dirname "$0")" && pwd)"
repository_name=$(basename $(dirname $DIR))
echo $repository_name $DIR
