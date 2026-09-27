for dir in `ls | grep *.service`
do
    printf "%-30s %s\n" [$dir]
    cp $dir /etc/systemd/system/
done
systemctl daemon-reload
for dir in `ls | grep *.service`
do
    systemctl restart $dir
done
printf "\033[0;32m\033[1m* Finish Update \033[0m\n"
