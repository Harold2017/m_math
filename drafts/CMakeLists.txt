# drafts
function(addDraft name src)
    add_executable(${name} ${src})
    math(EXPR lastIndex "${ARGC} - 1")
    foreach(index RANGE 2 ${lastIndex})
        target_link_libraries(${name} ${ARGV${index}})
    endforeach()
endfunction()

addExample(d_filter filter.cpp ${OpenCV_LIBS})
addExample(d_curvefit curvefit.cpp ${OpenCV_LIBS})
addExample(d_curvature curvature.cpp ${OpenCV_LIBS})
addExample(d_correlation correlation.cpp ${OpenCV_LIBS})
addExample(d_filter1d filter1d.cpp ${OpenCV_LIBS})