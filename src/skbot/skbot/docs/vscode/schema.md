---
title: using schema for sdf validation and tips
tags:
    - sdf
    - schema
    - xml
---
- Install Red Hat XML extension [...](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml)
- Download schema from sdformat.org
- Copy files to project / global location
- Associate file types to `<path>/root.xsd`

```json title="settings"
"xml.fileAssociations": [
        {
            "pattern": "**/*.sdf",
            "systemId": "${workspaceFolder}/vscode/sdf_schema/root.xsd"
        },
        {
            "pattern": "**/*.world",
            "systemId": "${workspaceFolder}/vscode/sdf_schema/root.xsd"
        }
    ]
```

---

# Resource
- [sdformat.org](http://sdformat.org/schemas/root.xsd)
- [Download schema offline](https://myxml.in/download-xsd-from-url.html)

