clear; clc;
dbstop if error;

%% set all initial parameters
initial_state = [-900, -400,0,0,0,0,2]; %[x,y,xd,yd,ux,uy,loc]
initial_diameter = [25,25,0,0,0,0]; %  hyperrectangle's radius
[initial_state(5:6),~] = getInputs(initial_state,initial_diameter); % calculate ux,uy
% Reach = [];
simulation_number = 0;

disp(initial_state(5:6));

global tol dim fulldim time_span guardPassive;
tol = 10^-12;   % tolerance
guardPassive = [120,125]; % interval of time that a transition to passive may occur (assume one transition must be made)
dim = 6; % 6D is [x,y,xd,yd,ux,uy], 4D is [x,y,xd,yd]
fulldim = length(initial_state);
time_span = [0,240]; % scaled in (min)

%% initialize 
start = 1; % index of starting cover
queue(start) = cover(initial_state,time_span(1),[],[],[],[],initial_diameter,dim,start,0);
lastID = 1;
head = queue(start).Next;

% Reach(1) = reachtube(4,initial_state,[],[],time_span(1),0);

%% safety verification
tic;  % Used to measure elapsed time, paired with 'toc'
flag = true;
while (flag)
    [queue(start),safeflag,unsafeflag] = computeReachtube(queue(start));

    disp('Simulating from:');
    disp(queue(start).x0);
    disp('with delta:');
    disp(queue(start).dia);
    simulation_number = simulation_number+1;
    disp('Simulation # ');
    disp(simulation_number);

    % Check safety of reachtube
    if safeflag
        disp('this trajactory is safe,go to next');
        % Prune leaves
        if start~=1 && all(queue(queue(queue(start).parent).children).isleaf) 
            parID = queue(queue(start).parent).ID;
            childID = queue(parID).children;
            queue(parID).copyCover(queue(childID(1)));
            for i=2:length(childID)
                queue(parID) = queue(parID).coverUnion(...
                    queue(childID(i)),1:length(queue(childID(i)).T));
                queue(childID(i)).removeNode;
                delete(queue(childID(i)));
            end
            queue(childID(1)).removeNode;
            delete(queue(childID(1)));

            % Recursive check for leaves
            while ~isempty(parID) && parID~=1
                parID = queue(queue(parID).parent).ID;
                if all(queue(queue(parID).children).isleaf)
                    childID = queue(parID).children;
                    queue(parID).copyCover(queue(childID(1)));
                    for i=2:length(childID)
                        queue(parID) = queue(parID).coverUnion(...
                            queue(childID(i)),1:length(queue(childID(i)).T));
                        queue(childID(i)).removeNode;
                        delete(queue(childID(i)));
                    end
                    queue(childID(1)).removeNode;
                    delete(queue(childID(1)));
                else
                    parID = [];
                end
            end
        end
            
        if isempty(head)
            safe = 1;
            flag = false;
            disp('all the partitions has been verified to be safe');
            
%             plotReach(queue(1),1:2); % plot results
        end
    else
        if unsafeflag
            flag = false;
            safe = 0;
            disp('finding one trajectory in the unsafe set');
        else
            if partitionNum >= partitionBnd
                disp('Max number of partitions reached. Unknown results.');
                safe = -1;
                break
            else
                disp('finding one suspect reachtube, partition it');
                [newA,newB] = cutset(queue(start).x0,queue(start).dia,2,...
                    dim,queue(start).t0); 
                
                % Reduces redundant partitions, such as when there are
                % zero-diameter dimensions
                [newA,ia,~] = unique(newA,'rows');
                newB = newB(ia,:);
                
                partitionNum = partitionNum + 1;
                % % add partitions to queue and remove cover
                lastID = lastID+1;
                queue(lastID) = cover(newA(1,:),queue(start).t0,...
                        [],[],[],[],newB(1,:),dim,lastID,queue(start).ID);
                queue(lastID).insertAfter(queue(start)); % insert to top of queue which is right after the last current cover
                head = queue(lastID); % update head pointer to top of queue

                % Update children
                queue(start).addChildren(lastID:(lastID+size(newA,1)-1));

                for i=2:size(newA,1)
                    lastID = lastID+1;
                    queue(lastID) = cover(newA(i,:),queue(start).t0,...
                        [],[],[],[],newB(i,:),dim,lastID,queue(start).ID);
                    queue(lastID).insertAfter(queue(lastID-1));
                end
            end
        end
    end % end post-simulation safety check

    % Verify next cover
    if flag
        start = head.ID;
        head = head.Next; % pointer to top of queue
    end
end % end verification no covers left to check
toc;  % Used to measure elapsed time, paired with 'tic'

% Reach = queue(1);

disp('number of simulations:');
disp(simulation_number);