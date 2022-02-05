# VSCode tips

## associations ext

```
"files.associations": {
    "*.xacro": "xml",
    "*.world": "xml",
    "sdf": "xml"
  }
```

## auto snippet

[Auto Snippet](https://marketplace.visualstudio.com/items?itemName=Gruntfuggly.auto-snippet)

```json title="snippet example"
"xacro_file_header": {
		"prefix": "xacro_header",
		"body": [
		  "<?xml version=\"1.0\"?>",
		  "<robot name=\"${1}\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">",
		  "</robot>"
		]
	}
```

```json
"autoSnippet.snippets": [
    {
        "pattern": "**/model.config",
        "snippet": "gazebo_model_config"
    },
    {
        "pattern": "**/model.sdf",
        "snippet": "gazebo_model_sdf"
    },
    {
        "pattern": "**/*.xacro",
        "snippet": "xacro_file_header"
    }
],

```

## 
```json
"emeraldwalk.runonsave": {
    "commands": [
        {
            "match": ".sdf.xacro",
            "cmd": "export PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages && /opt/ros/foxy/bin/xacro ${file} -o ${fileDirname}/${fileBasenameNoExt}"
        }
    ]
}
```

## Extensions
### Draw.io integration
- [marketplace](https://marketplace.visualstudio.com/items?itemName=hediet.vscode-drawio)