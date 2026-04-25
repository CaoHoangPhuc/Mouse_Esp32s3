param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$OutputFile = "NOTEBOOKLM_PROJECT_EXPORT.md"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$resolvedRoot = (Resolve-Path -LiteralPath $ProjectRoot).Path
$outputPath = Join-Path $resolvedRoot $OutputFile

$excludedDirectoryNames = @(
    ".git",
    "logs"
)

$includedFileNames = @(
    ".gitignore"
)

$includedExtensions = @(
    ".ino",
    ".h",
    ".hpp",
    ".c",
    ".cpp",
    ".md",
    ".txt",
    ".ps1",
    ".json",
    ".yml",
    ".yaml",
    ".ini",
    ".cfg"
)

function Test-IsExcludedPath {
    param(
        [string]$FullName
    )

    $relativePath = Get-RelativePath -BasePath $resolvedRoot -TargetPath $FullName
    $segments = $relativePath -split "[/\\]"

    foreach ($segment in $segments) {
        if ($excludedDirectoryNames -contains $segment) {
            return $true
        }
    }

    return $false
}

function Get-RelativePath {
    param(
        [string]$BasePath,
        [string]$TargetPath
    )

    $baseUri = [System.Uri]((Resolve-Path -LiteralPath $BasePath).Path.TrimEnd('\') + '\')
    $targetUri = [System.Uri](Resolve-Path -LiteralPath $TargetPath).Path
    return [System.Uri]::UnescapeDataString($baseUri.MakeRelativeUri($targetUri).ToString()).Replace("\", "/")
}

function Test-IsIncludedFile {
    param(
        [System.IO.FileInfo]$File
    )

    if ($File.FullName -eq $outputPath) {
        return $false
    }

    if (Test-IsExcludedPath -FullName $File.FullName) {
        return $false
    }

    if ($includedFileNames -contains $File.Name) {
        return $true
    }

    return $includedExtensions -contains $File.Extension.ToLowerInvariant()
}

function Get-CodeFenceLanguage {
    param(
        [string]$RelativePath
    )

    $extension = [System.IO.Path]::GetExtension($RelativePath).ToLowerInvariant()

    switch ($extension) {
        ".ino" { return "cpp" }
        ".c" { return "c" }
        ".cpp" { return "cpp" }
        ".h" { return "cpp" }
        ".hpp" { return "cpp" }
        ".md" { return "md" }
        ".txt" { return "text" }
        ".ps1" { return "powershell" }
        ".json" { return "json" }
        ".yml" { return "yaml" }
        ".yaml" { return "yaml" }
        ".ini" { return "ini" }
        ".cfg" { return "ini" }
        default { return "" }
    }
}

function Get-TreeLines {
    param(
        [string[]]$RelativePaths
    )

    $lines = New-Object System.Collections.Generic.List[string]
    $seenDirectories = New-Object System.Collections.Generic.HashSet[string]

    foreach ($relativePath in $RelativePaths) {
        $parts = $relativePath -split "[/\\]"

        if ($parts.Length -gt 1) {
            for ($i = 0; $i -lt ($parts.Length - 1); $i++) {
                $directoryKey = ($parts[0..$i] -join "/")
                if ($seenDirectories.Add($directoryKey)) {
                    $lines.Add(("  " * $i) + $parts[$i] + "/")
                }
            }
        }

        $fileIndent = if ($parts.Length -gt 1) { "  " * ($parts.Length - 1) } else { "" }
        $lines.Add($fileIndent + $parts[-1])
    }

    return $lines
}

$files = Get-ChildItem -LiteralPath $resolvedRoot -Recurse -File |
    Where-Object { Test-IsIncludedFile -File $_ } |
    Sort-Object FullName

$relativePaths = @(
    foreach ($file in $files) {
        Get-RelativePath -BasePath $resolvedRoot -TargetPath $file.FullName
    }
)

$treeLines = Get-TreeLines -RelativePaths $relativePaths
$generatedAt = Get-Date -Format "yyyy-MM-dd HH:mm:ss zzz"

$builder = New-Object System.Text.StringBuilder

[void]$builder.AppendLine("# NotebookLM Project Export")
[void]$builder.AppendLine()
[void]$builder.AppendLine('Generated from the Arduino project at "' + $resolvedRoot + '" on ' + $generatedAt + '.')
[void]$builder.AppendLine()
[void]$builder.AppendLine("This bundle is intended to be imported as a single source into NotebookLM. It includes the project tree and the full contents of the selected text/code files.")
[void]$builder.AppendLine()
[void]$builder.AppendLine("Included file count: $($files.Count)")
[void]$builder.AppendLine()
[void]$builder.AppendLine('Excluded directories: `.git`, `logs`')
[void]$builder.AppendLine()
[void]$builder.AppendLine("## Project Tree")
[void]$builder.AppendLine()
[void]$builder.AppendLine('```text')
foreach ($line in $treeLines) {
    [void]$builder.AppendLine($line)
}
[void]$builder.AppendLine('```')
[void]$builder.AppendLine()
[void]$builder.AppendLine("## Full File Contents")
[void]$builder.AppendLine()

foreach ($file in $files) {
    $relativePath = Get-RelativePath -BasePath $resolvedRoot -TargetPath $file.FullName
    $language = Get-CodeFenceLanguage -RelativePath $relativePath
    $content = Get-Content -LiteralPath $file.FullName -Raw

    [void]$builder.AppendLine('### `' + $relativePath + '`')
    [void]$builder.AppendLine()
    [void]$builder.AppendLine('````' + $language)
    [void]$builder.Append($content)
    if (-not $content.EndsWith("`n") -and -not $content.EndsWith("`r")) {
        [void]$builder.AppendLine()
    }
    [void]$builder.AppendLine('````')
    [void]$builder.AppendLine()
}

[System.IO.File]::WriteAllText($outputPath, $builder.ToString(), [System.Text.UTF8Encoding]::new($false))
Write-Output "Created $outputPath"
