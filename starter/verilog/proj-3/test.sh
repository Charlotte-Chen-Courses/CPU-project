#!/bin/bash
echo "Generating ground truth outputs from original processor" 
cd golden_model

# This only runs *.s files. How could you add *.c files?
for source_file in programs/*.s programs/*.c; do
    if [ "$source_file" = "programs/crt.s" ]
    then 
        continue 
    fi 
    program=$(echo "$source_file" | cut -d '.' -f1 | cut -d '/' -f 2) 

    echo "Running $program"
    make $program.out > /dev/null
done

echo "Comparing ground truth outputs to new processor"

cd ..

# Initialize counters
total_tests=0
passed_tests=0

for source_file in programs/*.s programs/*.c; do
    program=$(echo "$source_file" | cut -d '.' -f1 | cut -d '/' -f 2) 
    
    if [ "$source_file" = "programs/crt.s" ]; then
        continue 
    fi 

    echo "Running $program"
    make "$program.out" > /dev/null

    echo "Comparing writeback output for $program"
    diff "output/$program.wb" "golden_model/output/$program.wb" > /dev/null
    writeback_diff=$?

    echo "Comparing memory output for $program"
    grep "^@@@" "output/$program.out" > "output/tmp_$program.out"
    grep "^@@@" "golden_model/output/$program.out" > "output/golden_$program.out"
    diff "output/tmp_$program.out" "output/golden_$program.out" > /dev/null
    memback_diff=$?
    rm -f "output/tmp_$program.out" "output/golden_$program.out"

    # Increment test count
    total_tests=$((total_tests + 1))

    echo "Printing Passed or Failed"
    if [[ "$writeback_diff" -eq 0 && "$memback_diff" -eq 0 ]]; then
        echo "$program: PASSED"
        passed_tests=$((passed_tests + 1))  
    else
        echo "$program: FAILED"
    fi

    echo "--------------------------------------"
done


# Print summary
echo "Test Summary: $passed_tests/$total_tests tests passed."

echo "Cleaning up generated files..."
make nuke > /dev/null
cd golden_model
make nuke > /dev/null
cd ..
echo "All generated files have been removed."
