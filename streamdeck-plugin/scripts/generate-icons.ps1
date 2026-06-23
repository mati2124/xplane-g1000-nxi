param(
  [string[]]$Paths,
  [int]$Size = 144
)

Add-Type -AssemblyName System.Drawing

function New-IconPng {
  param([string]$OutputPath, [int]$Size)

  $bmp = New-Object System.Drawing.Bitmap $Size, $Size
  $g = [System.Drawing.Graphics]::FromImage($bmp)
  $g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
  $g.Clear([System.Drawing.Color]::FromArgb(255, 16, 24, 32))

  $penWidth = [Math]::Max(6, $Size / 16)
  $pen = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(255, 0, 255, 136)), $penWidth
  $margin = [Math]::Floor($Size * 0.18)
  $g.DrawArc($pen, $margin, $margin, $Size - 2 * $margin, $Size - 2 * $margin, 40, 280)
  $g.DrawLine($pen, [Math]::Floor($Size * 0.68), [Math]::Floor($Size * 0.30), [Math]::Floor($Size * 0.68), [Math]::Floor($Size * 0.54))
  $g.DrawLine($pen, [Math]::Floor($Size * 0.68), [Math]::Floor($Size * 0.54), [Math]::Floor($Size * 0.50), [Math]::Floor($Size * 0.54))

  $bmp.Save($OutputPath, [System.Drawing.Imaging.ImageFormat]::Png)
  $g.Dispose(); $bmp.Dispose(); $pen.Dispose()
}

foreach ($base in $Paths) {
  New-IconPng -OutputPath "$base.png" -Size $Size
  New-IconPng -OutputPath "$base@2x.png" -Size ($Size * 2)
}
