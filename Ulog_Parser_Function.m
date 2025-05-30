%Title: Pixhawk ULOG Data Parser
%Author: Ross W. Heidersbach
%Date: 07/25/2022

%Revision: 1.0

%This Function Converts a Raw ULOG File to a MATLAB Data Structure. A User is
%Required to Supply the File Path of the ULOG File of Interest to the
%Function

%%

function logData = Ulog_Parser_Function(file, folder)

%Define Alphabet Array
Alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';

%Load Pixhawk ULOG Data File
ULOG_Directory = ulogreader(fullfile(folder, file));

%Read All Pixhawk ULOG Topic Messages
ULOG_Messages = readTopicMsgs(ULOG_Directory);

%For-Loop Executable per Each ULOG Topic
for il = 1:1:height(ULOG_Messages)

    %Isolate Current Topic Name
    Topic_Name = ULOG_Messages{il,1};

    %Isolate Current Instance ID Number
    Instance_ID_Number = ULOG_Messages{il,2};

    %Convert Current Instance ID Number to Equivalent Letter (Needed to Dynamically Name Structure Fields)
    Instance_ID_Letter = Alphabet(Instance_ID_Number+1);

    %Isolate Current Data Time Table
    Data_Time_Table = ULOG_Messages{il,5};

    %Convert Current Data Time Table to Table
    Data_Table = timetable2table(Data_Time_Table{1,1});

    %For-Loop Executable per Each Column of Data Table
    for jl = 1:1:width(Data_Table)

        %Isolate Current Column Header Title (aka Variable Name)
        Variable_Name = Data_Table.Properties.VariableNames{jl};

        %If-Statement Executable if Variable Name String Contains "." or "[]"
        if (contains(Variable_Name,'.')) || (contains(Variable_Name,'[')) || (contains(Variable_Name,']'))

            %Replace "." with "_"
            Variable_Name = strrep(Variable_Name,".","_");

            %Replace "[" with "_"
            Variable_Name = strrep(Variable_Name,"[","_");

            %Replace "]" with "_"
            Variable_Name = strrep(Variable_Name,"]","_");
        end

        %Extract Current Variable Data and Save to the Appropriate Structure Field
        logData.(Topic_Name).(Instance_ID_Letter).(Variable_Name) = Data_Table{:,jl};

    end
end

end
