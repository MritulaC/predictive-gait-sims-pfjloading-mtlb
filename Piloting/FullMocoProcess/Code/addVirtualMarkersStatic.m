%% This function adds virtual markers to the static trial at specific points
%  to assist in the use of scaling.
%
%  Hip joint centres are placed according to the method outlined by
%  Harrington et al. (2007), J Biomech, 40: 595-602. 
%
%  Knee joint centres are placed at the mid point of the femoral epicondyle
%  markers.
%
%  Pseudo ankle joint centres are placed at the mid points of the malleoli
%  markers.
%
%  A marker at the mid point of the two metatarsal markers.
%
%  Markers around the foot and ankle are projected to the floor to assist
%  in aligning the scaling parameters with the axis they are relevant for
%  scaling.
%
%  Markers at the middle of the set of upper torso and pelvis markers used
%  to help scale torso and pelvis length.
%
%  Note that the application of this is only relevant to the markerset used
%  with this data. Different labelling of markers will require adaptating
%  this function to make it work.
%
%  @author: Aaron Fox
%  Centre for Sport Research, Deakin University
%  aaron.f@deakin.edu.au

    % % % %scaling can work as
    % % % femur = HJC to KJC
    % % % shank = KJC to AJC
    % % % feet body height = ajc to floor AJC
    % % % feet length = ajc floor to mid MT floor
    % % % feet width = MT1 floor to MT5 floor

%%

function addVirtualMarkersStatic(staticTRC,outputTRC)
    
    %Function input checks and organising
    if nargin < 1
        error('At least one input of the static .trc file is required.')
    end
    if nargin < 2
        [~,name,~] = fileparts(staticTRC);
        outputTRC = [name,'_withVirtualMarkers.trc'];
    end
    
    %Import opensim libraries
    import org.opensim.modeling.*
    
    %% Load in the trc file
    
    % Use the Vec3 TimeSeriesTable to read the Vec3 type data file.
    staticTable = TimeSeriesTableVec3(staticTRC);
    % Use the OpenSim Utility function, osimTable2Struct to
    % convert the OpenSim table into a Matlab Struct for ease of use.
    staticMarkerData = osimTableToStruct(staticTable);
    
    %% Calculate hip joint centre locations

    %Get ASIS and PSIS data
    %In this step we also convert back to a traditional Vicon coordinate
    %system from the opensim system. It was easier to do this than mess
    %with the hip joint centre calculations, and then convert the HJC
    %locations later.
    RASIS = [staticMarkerData.RASI(:,3)*1000,...
        staticMarkerData.RASI(:,1)*1000,...
        staticMarkerData.RASI(:,2)*1000]';
    LASIS = [staticMarkerData.LASI(:,3)*1000,...
        staticMarkerData.LASI(:,1)*1000,...
        staticMarkerData.LASI(:,2)*1000]';
    RPSIS = [staticMarkerData.RPSI(:,3)*1000,...
        staticMarkerData.RPSI(:,1)*1000,...
        staticMarkerData.RPSI(:,2)*1000]';
    LPSIS = [staticMarkerData.LPSI(:,3)*1000,...
        staticMarkerData.LPSI(:,1)*1000,...
        staticMarkerData.LPSI(:,2)*1000]';
   
    %Calculate hip joint centre at each time step
    for t=1:size(RASIS,2)

        %Right-handed Pelvis reference system definition 
        SACRUM(:,t)=(RPSIS(:,t)+LPSIS(:,t))/2; 
        
        %Global Pelvis Center position
        OP(:,t)=(LASIS(:,t)+RASIS(:,t))/2;    

        PROVV(:,t)=(RASIS(:,t)-SACRUM(:,t))/norm(RASIS(:,t)-SACRUM(:,t));  
        IB(:,t)=(RASIS(:,t)-LASIS(:,t))/norm(RASIS(:,t)-LASIS(:,t));    

        KB(:,t)=cross(IB(:,t),PROVV(:,t));                               
        KB(:,t)=KB(:,t)/norm(KB(:,t));

        JB(:,t)=cross(KB(:,t),IB(:,t));                               
        JB(:,t)=JB(:,t)/norm(JB(:,t));

        OB(:,t)=OP(:,t);

        %rotation+ traslation in homogeneous coordinates (4x4)
        pelvis(:,:,t)=[IB(:,t) JB(:,t) KB(:,t) OB(:,t);
                       0 0 0 1];

        %Trasformation into pelvis coordinate system (CS)
        OPB(:,t)=inv(pelvis(:,:,t))*[OB(:,t);1];    

        PW(t)=norm(RASIS(:,t)-LASIS(:,t));
        PD(t)=norm(SACRUM(:,t)-OP(:,t));

        %Harrington formulae (starting from pelvis center)
        diff_ap(t)=-0.24*PD(t)-9.9;
        diff_v(t)=-0.30*PW(t)-10.9;
        diff_ml(t)=0.33*PW(t)+7.3;

        %vector that must be subtract to OP to obtain hjc in pelvis CS
        vett_diff_pelvis_sx(:,t)=[-diff_ml(t);diff_ap(t);diff_v(t);1];
        vett_diff_pelvis_dx(:,t)=[diff_ml(t);diff_ap(t);diff_v(t);1];    

        %hjc in pelvis CS (4x4)
        rhjc_pelvis(:,t)=OPB(:,t)+vett_diff_pelvis_dx(:,t);  
        lhjc_pelvis(:,t)=OPB(:,t)+vett_diff_pelvis_sx(:,t);  


        %Transformation Local to Global
        RHJC(:,t)=pelvis(1:3,1:3,t)*[rhjc_pelvis(1:3,t)]+OB(:,t);
        LHJC(:,t)=pelvis(1:3,1:3,t)*[lhjc_pelvis(1:3,t)]+OB(:,t);

    end

    % [time x [x y z]]
    RHJC = RHJC';
    LHJC = LHJC';

    %Add HJC markers into matlab struct
    %Convert back to opensim coordinate system and m
    staticMarkerData.RHJC = [RHJC(:,2)/1000,RHJC(:,3)/1000,RHJC(:,1)/1000];
    staticMarkerData.LHJC = [LHJC(:,2)/1000,LHJC(:,3)/1000,LHJC(:,1)/1000];
    
    %% Create mid-epicondyle and malleoli markers
    
    %Mid epicondyles
    staticMarkerData.RKJC = (staticMarkerData.RLFC + staticMarkerData.RMFC) / 2;
    staticMarkerData.LKJC = (staticMarkerData.LLFC + staticMarkerData.LMFC) / 2;
    
    %Mid malleoli
    staticMarkerData.RAJC = (staticMarkerData.RLMAL + staticMarkerData.RMMAL) / 2;
    staticMarkerData.LAJC = (staticMarkerData.LLMAL + staticMarkerData.LMMAL) / 2;
    
    %Mid metatarsal
    staticMarkerData.RMT3 = (staticMarkerData.RMT1 + staticMarkerData.RMT5) / 2;
    staticMarkerData.LMT3 = (staticMarkerData.LMT1 + staticMarkerData.LMT5) / 2;
    
    %% Create projections of a number of foot and ankle markers down to the
    %  floor. This is achieved by shifting the y-value to be zero
    
    %Heel marker
    staticMarkerData.fRCAL = [staticMarkerData.RCAL(:,1),zeros(length(staticMarkerData.RCAL),1),staticMarkerData.RCAL(:,3)];
    staticMarkerData.fLCAL = [staticMarkerData.LCAL(:,1),zeros(length(staticMarkerData.LCAL),1),staticMarkerData.LCAL(:,3)];
    
    %Ankle joint centre
    staticMarkerData.fRAJC = [staticMarkerData.RAJC(:,1),zeros(length(staticMarkerData.RAJC),1),staticMarkerData.RAJC(:,3)];
    staticMarkerData.fLAJC = [staticMarkerData.LAJC(:,1),zeros(length(staticMarkerData.LAJC),1),staticMarkerData.LAJC(:,3)];
    
    %Toe markers
    staticMarkerData.fRMT1 = [staticMarkerData.RMT1(:,1),zeros(length(staticMarkerData.RMT1),1),staticMarkerData.RMT1(:,3)];
    staticMarkerData.fRMT5 = [staticMarkerData.RMT5(:,1),zeros(length(staticMarkerData.RMT5),1),staticMarkerData.RMT5(:,3)];
    staticMarkerData.fRMT3 = [staticMarkerData.RMT3(:,1),zeros(length(staticMarkerData.RMT3),1),staticMarkerData.RMT3(:,3)];
    staticMarkerData.fLMT1 = [staticMarkerData.LMT1(:,1),zeros(length(staticMarkerData.LMT1),1),staticMarkerData.LMT1(:,3)];
    staticMarkerData.fLMT5 = [staticMarkerData.LMT5(:,1),zeros(length(staticMarkerData.LMT5),1),staticMarkerData.LMT5(:,3)];
    staticMarkerData.fLMT3 = [staticMarkerData.LMT3(:,1),zeros(length(staticMarkerData.LMT3),1),staticMarkerData.LMT3(:,3)];
    
    %% Mid-points for torso and pelvis markers
    
    %Mid-torso
    staticMarkerData.MTOR = (((staticMarkerData.RACR + staticMarkerData.LACR) / 2) + ((staticMarkerData.C7 + staticMarkerData.CLAV) / 2)) / 2;
    
    %Mid-pelvis
    staticMarkerData.MPEL = (((staticMarkerData.RASI + staticMarkerData.LASI) / 2) + ((staticMarkerData.RPSI + staticMarkerData.LPSI) / 2)) / 2;

    %% Export to new trc file

    %Convert updated matlab struct back to osim table
    newMarkerTable = osimTableFromStruct(staticMarkerData);

    %Get the table meta data from the original table and add back into the new
    %one

    %Get meta data keys
    metaKeys = staticTable.getTableMetaDataKeys();

    %Loop through keys and append
    for ii = 0:metaKeys.size()-1
        %Get current meta data string
        currMeta = char(staticTable.getTableMetaDataKeys().get(ii));
        %Append with relevant value
        newMarkerTable.addTableMetaDataString(currMeta,...
            staticTable.getTableMetaDataAsString(currMeta));
        %Cleanup
        clear currMeta
    end

    %Write to file
    TRCFileAdapter().write(newMarkerTable,outputTRC);

    %Display termination message
    disp(['Virtual markers added to new .trc file: ',outputTRC]);

