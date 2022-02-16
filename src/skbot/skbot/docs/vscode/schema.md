---
title: using schema for sdf validation and tips
tags:
    - sdf
    - schema
    - xml
---
## Config XML ext. to validate xml against schema

- Install Red Hat XML extension [...](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml)
- Download schema from sdformat.org
  - root.xsd main entry point
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

## Add xacro support to schema

- Add xacro schema to support xacro tags and namespace 
- Add xacro support to `root.xsd`

```xml title="root.xsd" linenums="1" hl_lines="4 16 20"
<?xml version='1.0' encoding='UTF-8'?>
<xsd:schema 
  xmlns:xsd='http://www.w3.org/2001/XMLSchema'
  xmlns:xacro="http://www.ros.org/wiki/xacro"
>
  <xsd:annotation>
    <xsd:documentation xml:lang='en'>
      <![CDATA[SDF base element.]]>
    </xsd:documentation>
  </xsd:annotation>
  <xsd:include schemaLocation='/home/user/dev_ws/vscode/sdf/schema/types.xsd'/>
  <xsd:include schemaLocation='/home/user/dev_ws/vscode/sdf/schema/world.xsd'/>
  <xsd:include schemaLocation='/home/user/dev_ws/vscode/sdf/schema/model.xsd'/>
  <xsd:include schemaLocation='/home/user/dev_ws/vscode/sdf/schema/actor.xsd'/>
  <xsd:include schemaLocation='/home/user/dev_ws/vscode/sdf/schema/light.xsd'/>
  <xsd:import namespace="http://www.ros.org/wiki/xacro" schemaLocation="xacro.xsd"/>
  <xsd:element name='sdf'>
    <xsd:complexType>
      <xsd:choice maxOccurs='unbounded'>
        <xsd:element ref='xacro:include'/>
        <xsd:element ref='world'/>
        <xsd:element ref='model'/>
        <xsd:element ref='actor'/>
        <xsd:element ref='light'/>
      </xsd:choice>
      <xsd:attribute name='version' type='xsd:string' use='required' >
        <xsd:annotation>
          <xsd:documentation xml:lang='en'>
            <![CDATA[Version number of the SDF format.]]>
          </xsd:documentation>
        </xsd:annotation>
      </xsd:attribute>
    </xsd:complexType>
  </xsd:element>
</xsd:schema>

```

---

# Resource
- [sdformat.org](/home/user/dev_ws/vscode/sdf/schema/root.xsd)
- [Download schema offline](https://myxml.in/download-xsd-from-url.html)

