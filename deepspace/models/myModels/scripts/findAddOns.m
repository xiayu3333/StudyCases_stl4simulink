files = {'run.m'};
names = dependencies.toolboxDependencyAnalysis(files);

disp("The following toolboxes are required:")
disp(names)