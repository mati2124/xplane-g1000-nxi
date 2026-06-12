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
DisableProgramGroupPage=yes
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
Name: "startup"; Description: "Start the standalone app &automatically when I sign in to Windows"; GroupDescription: "Standalone shortcuts:"; Components: standalone; Flags: unchecked

[Registry]
Root: HKCU; Subkey: "Software\Microsoft\Windows\CurrentVersion\Run"; ValueType: string; ValueName: "{#MyAppName}"; ValueData: """{app}\avionics-standalone.exe"""; Components: standalone; Tasks: startup; Flags: uninsdeletevalue

[Files]
Source: "{#RepoRoot}\stage\plugin\xplane-avionics\win_x64\*"; DestDir: "{code:GetXPlanePluginsDir}\xplane-avionics\win_x64"; Components: plugin; Flags: ignoreversion
Source: "{#RepoRoot}\stage\plugin\xplane-avionics\assets\*"; DestDir: "{code:GetXPlanePluginsDir}\xplane-avionics\assets"; Components: plugin; Flags: ignoreversion recursesubdirs createallsubdirs

Source: "{#RepoRoot}\stage\standalone\*"; DestDir: "{app}"; Components: standalone; Flags: ignoreversion recursesubdirs createallsubdirs

; App icon used by the desktop / Start Menu shortcuts.
Source: "{#RepoRoot}\installer\assets\g1000-nxi.ico"; DestDir: "{app}"; Components: standalone; Flags: ignoreversion

[Icons]
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\avionics-standalone.exe"; WorkingDir: "{app}"; IconFilename: "{app}\g1000-nxi.ico"; Components: standalone; Tasks: desktopicon
Name: "{group}\{#MyAppName}"; Filename: "{app}\avionics-standalone.exe"; WorkingDir: "{app}"; IconFilename: "{app}\g1000-nxi.ico"; Components: standalone; Tasks: startmenu

[Run]
Filename: "{app}\avionics-standalone.exe"; Description: "Launch {#MyAppName}"; Flags: nowait postinstall skipifsilent unchecked; Components: standalone

[Code]
type
  TDisplayDeviceW = record
    cb: DWORD;
    DeviceName: array[0..31] of Char;
    DeviceString: array[0..127] of Char;
    StateFlags: DWORD;
    DeviceID: array[0..127] of Char;
    DeviceKey: array[0..127] of Char;
  end;
  TDevModeW = record
    dmDeviceName: array[0..31] of Char;
    dmSpecVersion: Word;
    dmDriverVersion: Word;
    dmSize: Word;
    dmDriverExtra: Word;
    dmFields: DWORD;
    dmPositionX: Longint;
    dmPositionY: Longint;
    dmDisplayOrientation: DWORD;
    dmDisplayFixedOutput: DWORD;
    dmColor: SmallInt;
    dmDuplex: SmallInt;
    dmYResolution: SmallInt;
    dmTTOption: SmallInt;
    dmCollate: SmallInt;
    dmFormName: array[0..31] of Char;
    dmLogPixels: Word;
    dmBitsPerPel: DWORD;
    dmPelsWidth: DWORD;
    dmPelsHeight: DWORD;
    dmDisplayFlags: DWORD;
    dmDisplayFrequency: DWORD;
    dmICMMethod: DWORD;
    dmICMIntent: DWORD;
    dmMediaType: DWORD;
    dmDitherType: DWORD;
    dmReserved1: DWORD;
    dmReserved2: DWORD;
    dmPanningWidth: DWORD;
    dmPanningHeight: DWORD;
  end;

const
  ENUM_CURRENT_SETTINGS = $FFFFFFFF;
  DISPLAY_DEVICE_ATTACHED_TO_DESKTOP = 1;
  DISPLAY_DEVICE_PRIMARY_DEVICE = 4;
  SM_CMONITORS = 80;

// Monitor enumeration for the Display Setup page (resolution + primary flag).
function EnumDisplayDevices(lpDevice: Cardinal; iDevNum: DWORD;
  var lpDisplayDevice: TDisplayDeviceW; dwFlags: DWORD): Boolean;
  external 'EnumDisplayDevicesW@user32.dll stdcall';
function EnumDisplaySettings(lpszDeviceName: String; iModeNum: DWORD;
  var lpDevMode: TDevModeW): Boolean;
  external 'EnumDisplaySettingsW@user32.dll stdcall';
function GetSystemMetrics(nIndex: Integer): Integer;
  external 'GetSystemMetrics@user32.dll stdcall';

var
  XPlanePage: TInputDirWizardPage;
  XPlaneDir: String;
  DisplayPage: TWizardPage;
  FullscreenCheck: TNewCheckBox;
  PfdCombo, MfdCombo: TNewComboBox;
  PfdLabel, MfdLabel, DisplayHelp: TNewStaticText;
  MonLabels: TArrayOfString;
  MonCount: Integer;

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

// The PFD/MFD monitor pickers only matter when full screen is enabled.
procedure UpdateDisplayControls();
begin
  PfdCombo.Enabled := FullscreenCheck.Checked;
  MfdCombo.Enabled := FullscreenCheck.Checked;
end;

procedure FullscreenCheckClick(Sender: TObject);
begin
  UpdateDisplayControls();
end;

// Enumerates the connected monitors with their current resolution and the
// primary flag, building "Monitor k  -  W x H  (primary)" labels. The list is
// ordered primary-first so the item index lines up with the app's monitor
// numbering (the in-app F key / --list-monitors let the user fix a mismatch).
procedure EnumerateMonitors();
var
  dd: TDisplayDeviceW;
  dm: TDevModeW;
  i, j, k, n, w, h, primaryIdx, src: Integer;
  name, item: String;
  labels: TArrayOfString;
  prims, order: array of Integer;
begin
  MonCount := 0;
  SetArrayLength(MonLabels, 0);
  SetArrayLength(labels, 16);
  SetArrayLength(prims, 16);
  n := 0;
  primaryIdx := -1;
  i := 0;
  while True do
  begin
    dd.cb := SizeOf(dd);
    if not EnumDisplayDevices(0, i, dd, 0) then
      Break;
    if (dd.StateFlags and DISPLAY_DEVICE_ATTACHED_TO_DESKTOP) <> 0 then
    begin
      name := '';
      for j := 0 to 31 do
      begin
        if dd.DeviceName[j] = #0 then
          Break;
        name := name + dd.DeviceName[j];
      end;

      w := 0;
      h := 0;
      dm.dmSize := SizeOf(dm);
      if EnumDisplaySettings(name, ENUM_CURRENT_SETTINGS, dm) then
      begin
        w := dm.dmPelsWidth;
        h := dm.dmPelsHeight;
      end;

      if n >= GetArrayLength(labels) then
      begin
        SetArrayLength(labels, n + 8);
        SetArrayLength(prims, n + 8);
      end;
      if (w > 0) and (h > 0) and (w < 32000) and (h < 32000) then
        labels[n] := IntToStr(w) + ' x ' + IntToStr(h)
      else
        labels[n] := '(unknown resolution)';
      if (dd.StateFlags and DISPLAY_DEVICE_PRIMARY_DEVICE) <> 0 then
      begin
        prims[n] := 1;
        primaryIdx := n;
      end
      else
        prims[n] := 0;
      n := n + 1;
    end;
    i := i + 1;
  end;

  if n = 0 then
  begin
    // Fallback if enumeration failed: number the monitors without resolution.
    MonCount := GetSystemMetrics(SM_CMONITORS);
    if MonCount < 1 then
      MonCount := 1;
    SetArrayLength(MonLabels, MonCount);
    for j := 0 to MonCount - 1 do
    begin
      if j = 0 then
        MonLabels[j] := 'Monitor 1  (primary)'
      else
        MonLabels[j] := Format('Monitor %d', [j + 1]);
    end;
    Exit;
  end;

  // Primary first, then the rest in enumeration order.
  SetArrayLength(order, n);
  k := 0;
  if primaryIdx >= 0 then
  begin
    order[0] := primaryIdx;
    k := 1;
  end;
  for j := 0 to n - 1 do
    if j <> primaryIdx then
    begin
      order[k] := j;
      k := k + 1;
    end;

  SetArrayLength(MonLabels, n);
  MonCount := n;
  for j := 0 to n - 1 do
  begin
    src := order[j];
    item := Format('Monitor %d  -  %s', [j + 1, labels[src]]);
    if prims[src] = 1 then
      item := item + '  (primary)';
    MonLabels[j] := item;
  end;
end;

procedure PopulateMonitors();
var
  i: Integer;
begin
  EnumerateMonitors();
  PfdCombo.Items.Clear();
  MfdCombo.Items.Clear();
  for i := 0 to MonCount - 1 do
  begin
    PfdCombo.Items.Add(MonLabels[i]);
    MfdCombo.Items.Add(MonLabels[i]);
  end;
  PfdCombo.ItemIndex := 0;
  if MonCount > 1 then
    MfdCombo.ItemIndex := 1
  else
    MfdCombo.ItemIndex := 0;
end;

procedure CreateDisplayPage();
begin
  DisplayPage := CreateCustomPage(XPlanePage.ID, 'Display Setup',
    'Choose how the PFD and MFD displays appear.');

  FullscreenCheck := TNewCheckBox.Create(DisplayPage);
  FullscreenCheck.Parent := DisplayPage.Surface;
  FullscreenCheck.Left := 0;
  FullscreenCheck.Top := ScaleY(8);
  FullscreenCheck.Width := DisplayPage.SurfaceWidth;
  FullscreenCheck.Height := ScaleY(20);
  FullscreenCheck.Caption :=
    'Run the displays full screen on dedicated monitors (no title bar)';
  FullscreenCheck.Checked := False;
  FullscreenCheck.OnClick := @FullscreenCheckClick;

  PfdLabel := TNewStaticText.Create(DisplayPage);
  PfdLabel.Parent := DisplayPage.Surface;
  PfdLabel.Left := 0;
  PfdLabel.Top := FullscreenCheck.Top + FullscreenCheck.Height + ScaleY(20);
  PfdLabel.Caption := 'PFD monitor:';

  PfdCombo := TNewComboBox.Create(DisplayPage);
  PfdCombo.Parent := DisplayPage.Surface;
  PfdCombo.Style := csDropDownList;
  PfdCombo.Left := ScaleX(100);
  PfdCombo.Top := PfdLabel.Top - ScaleY(3);
  PfdCombo.Width := DisplayPage.SurfaceWidth - ScaleX(100);

  MfdLabel := TNewStaticText.Create(DisplayPage);
  MfdLabel.Parent := DisplayPage.Surface;
  MfdLabel.Left := 0;
  MfdLabel.Top := PfdCombo.Top + PfdCombo.Height + ScaleY(14);
  MfdLabel.Caption := 'MFD monitor:';

  MfdCombo := TNewComboBox.Create(DisplayPage);
  MfdCombo.Parent := DisplayPage.Surface;
  MfdCombo.Style := csDropDownList;
  MfdCombo.Left := ScaleX(100);
  MfdCombo.Top := MfdLabel.Top - ScaleY(3);
  MfdCombo.Width := DisplayPage.SurfaceWidth - ScaleX(100);

  DisplayHelp := TNewStaticText.Create(DisplayPage);
  DisplayHelp.Parent := DisplayPage.Surface;
  DisplayHelp.Left := 0;
  DisplayHelp.Top := MfdCombo.Top + MfdCombo.Height + ScaleY(20);
  DisplayHelp.Width := DisplayPage.SurfaceWidth;
  DisplayHelp.AutoSize := False;
  DisplayHelp.Height := ScaleY(48);
  DisplayHelp.WordWrap := True;
  DisplayHelp.Caption :=
    'Each display fills its monitor; the 4:3 image is letterboxed to keep its '
    + 'shape.' + #13#10 +
    'You can change this any time in the app (press F to toggle full screen).';

  PopulateMonitors();
  UpdateDisplayControls();
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

  CreateDisplayPage();
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
  if PageID = DisplayPage.ID then
    Result := not IsComponentSelected('standalone');
end;

// Pre-seeds the standalone app's per-user settings file with the chosen full
// screen / monitor preferences. The keys mirror AppSettings.cpp; only those
// four lines are rewritten, so any other saved preferences are preserved.
procedure WriteDisplaySettings();
var
  path, dir, key: String;
  lines, newLines: TArrayOfString;
  i, n, fs, pfdIdx, mfdIdx: Integer;
begin
  if not IsComponentSelected('standalone') then
    Exit;

  dir := ExpandConstant('{userappdata}\XPlaneAvionics');
  ForceDirectories(dir);
  path := dir + '\settings.txt';

  n := 0;
  if LoadStringsFromFile(path, lines) then
  begin
    SetArrayLength(newLines, GetArrayLength(lines) + 4);
    for i := 0 to GetArrayLength(lines) - 1 do
    begin
      key := lines[i];
      if (Pos('pfdFullscreen=', key) <> 1) and
         (Pos('mfdFullscreen=', key) <> 1) and
         (Pos('pfdMonitor=', key) <> 1) and
         (Pos('mfdMonitor=', key) <> 1) then
      begin
        newLines[n] := lines[i];
        n := n + 1;
      end;
    end;
  end
  else
    SetArrayLength(newLines, 4);

  if FullscreenCheck.Checked then
    fs := 1
  else
    fs := 0;
  pfdIdx := PfdCombo.ItemIndex;
  if pfdIdx < 0 then
    pfdIdx := 0;
  mfdIdx := MfdCombo.ItemIndex;
  if mfdIdx < 0 then
    mfdIdx := 0;

  newLines[n] := 'pfdFullscreen=' + IntToStr(fs);
  newLines[n + 1] := 'mfdFullscreen=' + IntToStr(fs);
  newLines[n + 2] := 'pfdMonitor=' + IntToStr(pfdIdx);
  newLines[n + 3] := 'mfdMonitor=' + IntToStr(mfdIdx);
  SetArrayLength(newLines, n + 4);

  SaveStringsToFile(path, newLines, False);
end;

procedure CurStepChanged(CurStep: TSetupStep);
begin
  if CurStep = ssPostInstall then
    WriteDisplaySettings();
end;

function InitializeSetup(): Boolean;
begin
  Result := True;
end;
