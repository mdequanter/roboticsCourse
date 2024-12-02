name=$1
domain_id=$2
./build-package.sh $name
./launch-package.sh $name $domain_id