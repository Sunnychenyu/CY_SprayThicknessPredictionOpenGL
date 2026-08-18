############################################################
#   Get the cmake configure time and store it in variables
#   --  CompileTimeString
#       Format: Date: ${CMAKE_TIME_YEAR}-${CMAKE_TIME_MONTH}-${CMAKE_TIME_DAY}, Time: ${CMAKE_TIME_HOUR}:${CMAKE_TIME_MINUTE}:${CMAKE_TIME_SECOND}
#   --  CMAKE_TIME_YEAR
#   --  CMAKE_TIME_MONTH
#   --  CMAKE_TIME_DAY
#   --  CMAKE_TIME_HOUR
#   --  CMAKE_TIME_MINUTE
#   --  CMAKE_TIME_SECOND

#   Written by Tang Qing in Dec. 2023.
############################################################

message( STATUS "--------------\tCMake Time Setting\t--------------" )

#   Get year/month/day/hour/minute/second
string( TIMESTAMP COMPILE_TIME %Y%m%d%H%M%S )
#   To get the last two digits of the year, use %Y to get the full year.
string( TIMESTAMP CMAKE_TIME_YEAR %Y )            #   %y  The later 2 digits of year.
#   Get Month
string( TIMESTAMP CMAKE_TIME_MONTH %m )
#   Get Date
string( TIMESTAMP CMAKE_TIME_DAY %d )

message( STATUS "Compile time:${COMPILE_TIME}" )
message( STATUS "Year: ${CMAKE_TIME_YEAR}. Month: ${CMAKE_TIME_MONTH}. Day: ${CMAKE_TIME_DAY}." )

string( TIMESTAMP CMAKE_TIME_HOUR %H )            #   %y  The later 2 digits of year.
string( TIMESTAMP CMAKE_TIME_MINUTE %M )
string( TIMESTAMP CMAKE_TIME_SECOND %S )

string( APPEND CompileTimeString "Date: ${CMAKE_TIME_YEAR}-${CMAKE_TIME_MONTH}-${CMAKE_TIME_DAY}, Time: ${CMAKE_TIME_HOUR}:${CMAKE_TIME_MINUTE}:${CMAKE_TIME_SECOND}" )
message( STATUS "Compile Time: ${CompileTimeString}." )

message( STATUS "--------------\tCMake Time Setting\t--------------" )

