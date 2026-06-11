; G1000 NXi Windows installer (Inno Setup 6).
; CI invokes:
;   ISCC.exe /DMyAppVersion=x.y.z /DRepoRoot=... installer\windows\g1000nxi.iss

#ifndef MyAppVersion
  #define MyAppVersion "0.0.0"
#endif
#ifndef RepoRoot
  #define RepoRoot "..\.."
#endif

#define MyAppName "G1000 NXi"
#define MyAppPublisher "G1000 NXi"
#define MyAppURL "https://github.com/andywmm9-pixel/xplane-g1000-nxi"

[Setup]
AppId={{A7B3C4D5-E6F7-4890-ABCD-EF1234567890}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}/releases
AppUpdatesURL={#MyAppURL}/releases
DefaultDirName={autopf}\{#MyAppName}
DefaultGroupName={#MyAppName}
DisableProgramGroupDir=no
OutputDir=Output
OutputBaseFilename=g1000nxi-setup-{#MyAppVersion}
Compression=lzma2
SolidCompression=yes
ArchitecturesInstallIn64BitMode=x64
WizardStyle=modern
PrivilegesRequired=lowest

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Types]
Name: "full"; Description: "X-Plane plugin and standalone app"
Name: "custom"; Description: "Custom installation"; Flags: iscustom

[Components]
Name: "plugin"; Description: "X-Plane plugin (replaces the stock G1000 glass)"; Types: full custom; Flags: fixed
Name: "standalone"; Description: "Standalone desktop app (connects to X-Plane over the network)"; Types: full custom

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop shortcut for the standalone app"; GroupDescription: "Standalone shortcuts:"; Components: standalone
Name: "startmenu"; Description: "Create a &Start Menu shortcut"; GroupDescription: "Standalone shortcuts:"; Components: standalone; Flags: checkedonce

[Files]
Source: "{#RepoRoot}\stage\plugin\xplane-avionics\win_x64\*"; DestDir: "{code:GetXPlanePluginsDir}\xplane-avionics\win_x64"; Components: plugin; Flags: ignoreversion
Source: "{#RepoRoot}\stage\plugin\xplane-avionics\assets\*"; DestDir: "{code:GetXPlanePluginsDir}\xplane-avionics\assets"; Components: plugin; Flags: ignoreversion recursesubdirs createallsubdirs

Source: "{#RepoRoot}\stage\standalone\*"; DestDir: "{app}"; Components: standalone; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\avionics-standalone.exe"; WorkingDir: "{app}"; Components: standalone; Tasks: desktopicon
Name: "{group}\{#MyAppName}"; Filename: "{app}\avionics-standalone.exe"; WorkingDir: "{app}"; Components: standalone; Tasks: startmenu

[Run]
Filename: "{app}\avionics-standalone.exe"; Description: "Launch {#MyAppName}"; Flags: nowait postinstall skipifsilent unchecked; Components: standalone

[Code]
var
  XPlanePage: TInputDirWizardPage;
  XPlaneDir: String;

function DirLooksLikeXPlane(const Dir: String): Boolean;
begin
  Result := DirExists(Dir) and DirExists(Dir + '\Resources\plugins');
end;

function FindXPlaneInstall(): String;
var
  Steam: String;
begin
  Result := '';
  if RegQueryStringValue(HKCU, 'Software\Laminar Research\X-Plane', 'InstallPath', Result) then
    if DirLooksLikeXPlane(Result) then Exit else Result := '';
  if RegQueryStringValue(HKLM, 'Software\Laminar Research\X-Plane', 'InstallPath', Result) then
    if DirLooksLikeXPlane(Result) then Exit else Result := '';

  Steam := ExpandConstant('{autopf32}\Steam\steamapps\common\X-Plane 12');
  if DirLooksLikeXPlane(Steam) then
  begin
    Result := Steam;
    Exit;
  end;

  if DirLooksLikeXPlane('C:\X-Plane 12') then
    Result := 'C:\X-Plane 12'
  else if DirLooksLikeXPlane('D:\X-Plane 12') then
    Result := 'D:\X-Plane 12';
end;

function GetXPlanePluginsDir(Param: String): String;
begin
  Result := XPlaneDir + '\Resources\plugins';
end;

procedure InitializeWizard;
begin
  XPlaneDir := FindXPlaneInstall();
  XPlanePage := CreateInputDirPage(wpSelectComponents,
    'Select X-Plane 12 Folder', 'Where is X-Plane 12 installed?',
    'Choose the folder that contains X-Plane.exe and the Resources directory.' + #13#10 +
    'The plugin will be installed into Resources\plugins\xplane-avionics.',
    False, '');
  XPlanePage.Add('');
  if XPlaneDir <> '' then
    XPlanePage.Values[0] := XPlaneDir;
end;

function NextButtonClick(CurPageID: Integer): Boolean;
begin
  Result := True;
  if CurPageID = XPlanePage.ID then
  begin
    XPlaneDir := XPlanePage.Values[0];
    if not DirLooksLikeXPlane(XPlaneDir) then
    begin
      MsgBox('That folder does not look like an X-Plane 12 install.' + #13#10 +
             'It must contain a Resources\plugins directory.', mbError, MB_OK);
      Result := False;
    end;
  end;
end;

function ShouldSkipPage(PageID: Integer): Boolean;
begin
  Result := False;
  if PageID = XPlanePage.ID then
    Result := not IsComponentSelected('plugin');
end;

function InitializeSetup(): Boolean;
begin
  Result := True;
end;
