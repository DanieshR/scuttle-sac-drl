# Vendored JS

The dashboard needs **roslib.js** in this folder as `roslib.min.js`. It is vendored (not loaded
from a CDN) so the console works on the field laptop with no internet.

Fetch it once (run from a machine with internet, then it's committed/installed with the package):

```bash
curl -L -o roslib.min.js \
  https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js
```

If `roslib.min.js` is missing, `index.html` silently stays in **SIM mode** (a badge in the
top-right shows `○ SIM`), so the page still opens for design review — it just won't talk to ROS.
