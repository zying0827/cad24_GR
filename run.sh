if [ $# -eq 1 ]; then
    data=$1
    if [ $data -eq 4 ] || [ $data -eq 5 ]; then
        echo "case${data}"
        echo "./bin/router input/case${data}/case${data}_def/ input/case${data}/case${data}_cfg.json input/case${data}/case${data}.json output/chip.txt"
        ./bin/router input/case${data}/case${data}_def/ input/case${data}/case${data}_cfg.json input/case${data}/case${data}.json output/chip.txt
    else
        echo "Error"
        echo "only support [sh run.sh 4] or [sh run.sh 5]"
    fi
else
    echo "Error"
    echo "only support [sh run.sh 4] or [sh run.sh 5]"
fi