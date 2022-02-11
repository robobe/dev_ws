---
title: Extensions
tags:
    - vscode
    - extensions
---

# Extensions
- [Utils](#utils)
- [Draw and viewer](#draw-and-viewers)
- [ROS](#ros)
---

# Utils

![](/images/2022-02-11-12-34-43.png){ width=80, align=left}
### Auto snippets
Insert a snippet when opening an empty file
[Marketplace](https://marketplace.visualstudio.com/items?itemName=Gruntfuggly.auto-snippet)

#### Examples

Insert file template for gazebo sdf model config

```
"autoSnippet.snippets": [
    {
        "pattern": "**/model.config",
        "snippet": "gazebo_model_config"
    }
]
```
<details>
    <summary>gazebo_model_config snippet</summary>

```json title="snippet"
"gazebo_model_config": {
		"prefix": "gz_config",
		"body": [
		  "<?xml version=\"1.0\"?>",
		  "<model>",
		  "  <name>${1}</name>",
		  "  <version>1.0</version>",
		  "  <sdf version=\"${2|1.5,1.6|}\">${3:${1}}.sdf</sdf>",
		  "  <author>",
		  "    <name></name>",
		  "    <email></email>",
		  "  </author>",
		  "  <description>",
		  "  </description>",
		  "</model>"
		],
		"description": "gazebo model config file template"
	}
```
</details>

---
![](/images/2022-02-11-12-50-28.png){ width=80, align=left}
### Run on Save
Run commands when a file is saved in vscode.  
[marketplace](https://marketplace.visualstudio.com/items?itemName=emeraldwalk.RunOnSave)

#### Example
- Run xacro util to generate `urdf` or `sdf` files from `xacro`

```json title="run xacro"
"emeraldwalk.runonsave": {
    "commands": [
        {
            "match": ".sdf.xacro",
            "cmd": "export PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages && /opt/ros/foxy/bin/xacro ${file} -o ${fileDirname}/${fileBasenameNoExt}"
        }
    ]
}
```

---

## Draw and viewers
![](/images/2022-02-11-12-32-02.png){ width=80, align=left}
### Draw.io integration
- [marketplace](https://marketplace.visualstudio.com/items?itemName=hediet.vscode-drawio)

---

![](/images/2022-02-11-12-26-04.png){ width=80, align=left}
### Graphviz (gz files)
[Graphviz (dot) language support for Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=joaompinto.vscode-graphviz
)

- preview file generate by ros2 `tf2 treeview`

---

## ROS

![](/images/2022-02-11-12-29-41.png){ width=80, align=left}
### ROS2
[ROS2 nonanonno](https://marketplace.visualstudio.com/items?itemName=nonanonno.vscode-ros2)

---

![](/images/2022-02-11-18-18-31.png){ width=80, align=left}
### XML
XML Language Support by Red Hat
[Marketplace](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml)

