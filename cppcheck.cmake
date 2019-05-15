# additional target to perform cppcheck run, requires cppcheck

add_custom_target(
        cppcheck
        COMMAND cppcheck.exe
        --output-file=cppcheck_report.xml
        --xml
        --project=compile_commands.json
        --enable=warning,performance,portability,information,missingInclude
        #--std=c++11
        #--library=qt.cfg
        #--template="[{severity}][{id}] {message} {callstack} \(On {file}:{line}\)"
        #--verbose
        #--quiet
        ${ALL_SOURCE_FILES}
)