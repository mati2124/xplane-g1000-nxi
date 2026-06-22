param([string]$InputPath, [string]$OutputPath)
Add-Type -AssemblyName System.Drawing
$text = Get-Content -Path $InputPath -TotalCount 3
$dims = $text[1] -split "\s+"
$width = [int]$dims[0]
$height = [int]$dims[1]
$bytes = [System.IO.File]::ReadAllBytes($InputPath)
$pos = 0
while ($pos -lt $bytes.Length -and $bytes[$pos] -ne 0x0A) { $pos++ }
$pos++
while ($pos -lt $bytes.Length -and $bytes[$pos] -ne 0x0A) { $pos++ }
$pos++
while ($pos -lt $bytes.Length -and $bytes[$pos] -ne 0x0A) { $pos++ }
$pos++
$bmp = New-Object System.Drawing.Bitmap $width, $height
$idx = $pos
for ($y = 0; $y -lt $height; $y++) {
  for ($x = 0; $x -lt $width; $x++) {
    $r = $bytes[$idx++]
    $g = $bytes[$idx++]
    $b = $bytes[$idx++]
    $bmp.SetPixel($x, $y, [System.Drawing.Color]::FromArgb($r, $g, $b))
  }
}
$bmp.Save($OutputPath, [System.Drawing.Imaging.ImageFormat]::Png)
$bmp.Dispose()
