function [] = mex_LpC_sfunc(copyHeaders, sfuncName)

arguments
  copyHeaders logical = false;
  sfuncName char = 'LpC_Scaled_wrapper_sfunc';
end

generateLib = true;

% generate the static C library from the LpC classes
codegen_LpC_lib(generateLib, copyHeaders);

% static lib extension by OS
if ispc
  libext = 'lib';
elseif ismac
  libext = 'a';
elseif isunix
  libext = 'a';
end

ipath = ['-I' fullfile(pwd,'codegen','lib','LpC_wrapper')];
%libpath = ['-L' fullfile(pwd,'codegen','lib','LpC_wrapper')];
libdir = fullfile(pwd,'codegen','lib','LpC_wrapper');
libpath = ['-L' libdir];

%libfile = sprintf('LpC_wrapper.%s', libext);
libfile = sprintf('%s/LpC_wrapper.%s', libdir, libext);
%libs = ['-l' libfile];

outdir = fullfile(pwd,'vehicles','Lift+Cruise','obj');

% srcfile = './vehicles/Lift+Cruise/AeroProp/SFunction/codegen_sfunc/LpC_Scaled_wrapper_sfunc.c';
srcdir  = './vehicles/Lift+Cruise/AeroProp/SFunction/codegen_sfunc';
srcfile = fullfile(srcdir, [sfuncName '.c']);

if ~isfile(srcfile)
    error('mex_LpC_sfunc:MissingSource', ...
        'Source file not found: %s', srcfile);
end

%mex(ipath, libpath, libs, '-outdir', outdir, srcfile);
%mex(ipath, libpath, '-outdir', outdir, srcfile, libfile);
mex('-v','COMPFLAGS="$COMPFLAGS /MT"', ipath, libpath, '-outdir', outdir, srcfile, libfile, '-output', sfuncName);

end
