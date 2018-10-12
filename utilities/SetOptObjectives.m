%   Copyright (C) 2018-present, Facebook, Inc.
% 
%   This program is free software; you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation; version 2 of the License.
% 
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
% 
%   You should have received a copy of the GNU General Public License along
%   with this program; if not, write to the Free Software Foundation, Inc.,
%   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
%
%
%   Configure the objective array for the systune command based on the input
%   objective array:
Objectives_All = [] ;
LoopStruct_list={};

if ( contains(optOptions.OptConfig,'ClimbRateLoop'))
    ClimbRateLoopOptObjectives ;
    
    if( optOptions.OptviewSpec == 'On' )
        %check the requirements
        figure(237);viewSpec(Objectives_ClimbRateLoop);
    end
    
    Objectives_All = [Objectives_All, Objectives_ClimbRateLoop] ;
    LoopStruct_list=[LoopStruct_list, ClimbRateLoop];

end


if (contains(optOptions.OptConfig,'TASLoop'))
    TASLoopOptObjectives ;
    
    if( optOptions.OptviewSpec == 'On' )
        %check the requirements
        figure(238);viewSpec(Objectives_TASLoop);
    end

    Objectives_All = [Objectives_All, Objectives_TASLoop] ;
    LoopStruct_list=[LoopStruct_list, TASLoop];
end

if (contains(optOptions.OptConfig,'PitchLoop'))
    PitchLoopOptObjectives ;
    
    if( optOptions.OptviewSpec == 'On' )
        %check the requirements
        figure(236);viewSpec(Objectives_PitchLoop);
    end

    Objectives_All = [Objectives_All, Objectives_PitchLoop] ;
    LoopStruct_list=[LoopStruct_list, PitchLoop];

end

if (contains(optOptions.OptConfig,'AltLoop'))
    AltLoopOptObjectives ;
    
    if( optOptions.OptviewSpec == 'On' )
        %check the requirements
        figure(240);viewSpec(Objectives_AltLoop);
    end

    Objectives_All = [Objectives_All, Objectives_AltLoop] ;
    LoopStruct_list=[LoopStruct_list, AltLoop];

end

if (contains(optOptions.OptConfig,'PwrLoop'))
    PwrLoopOptObjectives ;
    
    if( optOptions.OptviewSpec == 'On' )
        %check the requirements
        figure(241);viewSpec(Objectives_PwrLoop);
    end

    Objectives_All = [Objectives_All, Objectives_PwrLoop] ;
    LoopStruct_list=[LoopStruct_list, PwrLoop];

end

if (contains(optOptions.OptConfig,'RollLoop'))
    RollLoopOptObjectives ;
    
    if( optOptions.OptviewSpec == 'On' )
        %check the requirements
        figure(242);viewSpec(Objectives_RollLoop);
    end

    Objectives_All = [Objectives_All, Objectives_RollLoop] ;
    LoopStruct_list=[LoopStruct_list, RollLoop];

end

if (contains(optOptions.OptConfig,'GndTrkLoop'))
    GndTrkLoopOptObjectives ;
    
    if( optOptions.OptviewSpec == 'On' )
        %check the requirements
        figure(243);viewSpec(Objectives_GndTrkLoop);
    end

    Objectives_All = [Objectives_All, Objectives_GndTrkLoop] ;
    LoopStruct_list=[LoopStruct_list, GndTrkLoop];

end

if (contains(optOptions.OptConfig,'YawRateLoop'))
    YawRateLoopOptObjectives ;
    
    if( optOptions.OptviewSpec == 'On' )
        %check the requirements
        figure(244);viewSpec(Objectives_YawRateLoop);
    end

    Objectives_All = [Objectives_All, Objectives_YawRateLoop] ;
    LoopStruct_list=[LoopStruct_list, YawRateLoop];

end


if (contains(optOptions.OptConfig,'FlexLoop'))
    FlexLoopOptObjectives ;
    
    if( optOptions.OptviewSpec == 'On' )
        %check the requirements
        figure(245);viewSpec(Objectives_FlexLoop);
    end

    Objectives_All = [Objectives_All, Objectives_FlexLoop] ;
    LoopStruct_list=[LoopStruct_list, FlexLoop];

end

if (contains(optOptions.OptConfig,'GLALoop'))
    GLALoopOptObjectives ;
    
    if( optOptions.OptviewSpec == 'On' )
        %check the requirements
        figure(245);viewSpec(Objectives_GLALoop);
    end

    Objectives_All = [Objectives_All, Objectives_GLALoop] ;
    LoopStruct_list=[LoopStruct_list, GLALoop];

end

% Report an error if the Objectives_All is empty
if ( isempty(Objectives_All) )
    warn('The optimization objectives must be defined first')
    return
end
