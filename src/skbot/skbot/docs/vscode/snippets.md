---
title: snippets
tags:
    - mkdocs
    - gazebo
    - sdf
---

- Gazebo
  * sdf
- ROS
  * urdf
- mkdocs
- xacro

---

# XACRO

```json title="xacro.code-snippet demo"
"xacro property": {
		"scope": "xml, xacro, sdf.xaco,",
		"prefix": "xacro_prop",
		"body": [
			"<xacro:property name=\"$1\" value=\"$2\" />"
		],
		"description": "Add xacro property"
	}
```