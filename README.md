# valkyrie_translator
ROS Control module accpeting torque commands via LCM and sending sensor measurements via LCM.

## Code Style

```
find -regextype egrep -regex '.*\.[ch](pp)?$' -exec astyle '{}' --style=allman --indent=spaces=2 --pad-oper --unpad-paren --pad-header --convert-tabs \;
```
