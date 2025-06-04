$(document).ready(function () {
  // Handle navigation
  $(".list-group-item").click(function (e) {
    e.preventDefault();

    // Update active states
    $(".list-group-item").removeClass("active");
    $(this).addClass("active");

    // Show selected section
    $(".content-section").removeClass("active");
    $($(this).attr("href")).addClass("active");
  });
});

$(document).ready(function () {
  // Show confirmation modal on restart button click
  $("#restartButton").click(function () {
    $("#restartModal").modal("show");
  });

  // Handle confirm restart
  $("#confirmRestartButton").click(function () {
    // Add actual restart logic here
    console.log("System restart confirmed");
    $("#restartModal").modal("hide");
  });
});

document.getElementById("dhcpEnabled").addEventListener("change", function () {
  const staticIp = document.getElementById("staticIp");
  const subnetMask = document.getElementById("subnetMask");
  const gateway = document.getElementById("gateway");
  const updateButton = document.getElementById("updateIpButton");

  staticIp.disabled = this.checked;
  subnetMask.disabled = this.checked;
  gateway.disabled = this.checked;
});

document
  .getElementById("checkUpdateButton")
  .addEventListener("click", function () {
    const updateStatus = document.getElementById("updateStatus");
    const checkButton = document.getElementById("checkUpdateButton");

    // Show checking status
    updateStatus.innerHTML =
      '<span class="badge bg-info">Checking for updates...</span>';

    // Simulate version check (replace with actual version check)
    setTimeout(() => {
      const currentVersion = "1.0.0";
      const latestVersion = "1.1.0";

      if (latestVersion > currentVersion) {
        // Update available
        updateStatus.innerHTML = `<span class="badge bg-warning">Update available: v${latestVersion}</span>`;
        checkButton.innerHTML =
          '<i class="bi bi-github me-2"></i>Update from GitHub';
        checkButton.classList.remove("btn-info");
        checkButton.classList.add("btn-warning");
      } else {
        // No update needed
        updateStatus.innerHTML =
          '<span class="badge bg-success">System is up to date</span>';
        checkButton.innerHTML =
          '<i class="bi bi-search me-2"></i>Check for Updates';
        checkButton.classList.remove("btn-primary");
        checkButton.classList.add("btn-info");
      }
    }, 1000); // Simulated delay for the check
  });

document.getElementById("estopButton").addEventListener("click", function () {
  const button = this;
  const isActive = button.classList.contains("btn-danger");

  if (isActive) {
    // E-Stop Released
    button.classList.remove("btn-danger");
    button.classList.add("btn-success");
    button.style.animation = "";
  } else {
    // E-Stop Activated
    button.classList.remove("btn-success");
    button.classList.add("btn-danger");
    button.style.animation = "pulse 2s infinite";
  }
});

async function fetchUptime() {
  try {
    const response = await fetch("/uptime");
    if (!response.ok) {
      throw new Error(`Response status: ${response.status}`);
    }

    const json = await response.json();
    const uptime = document.getElementById("uptime");
    const seconds = Math.floor(json / 1000);
    const days = Math.floor(seconds / (24 * 3600));
    const hours = Math.floor((seconds % (24 * 3600)) / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const remainingSeconds = seconds % 60;

    uptime.innerHTML = `${days}d ${hours}h ${minutes}m ${remainingSeconds}s`;
  } catch (error) {
    console.error(error.message);
  }
}

async function fetchVersion() {
  try {
    const response = await fetch("/version");
    if (!response.ok) {
      throw new Error(`Response status: ${response.status}`);
    }

    const json = await response.json();
    const version = document.getElementById("firmwareVersion");
    version.innerHTML = json.version;
  } catch (error) {
    console.error(error.message);
  }
}

async function fetchSSID() {
  try {
    const response = await fetch("/ssid");
    if (!response.ok) {
      throw new Error(`Response status: ${response.status}`);
    }

    const json = await response.json();
    const ssidElement = document.getElementById("currentSsid"); // Correct element ID
    ssidElement.innerHTML = json.ssid;
  } catch (error) {
    console.error(error.message);
  }
}

document
  .getElementById("saveNetworkSettingsButton")
  .addEventListener("click", async function (e) {
    e.preventDefault();

    const ssidInput = document.getElementById("ssid").value;
    const passwordInput = document.getElementById("password").value;

    console.log("Saving network settings:", {
      ssid: ssidInput,
      password: passwordInput,
    });

    try {
      const response = await fetch("/update_network", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          ssid: ssidInput,
          password: passwordInput,
        }),
      });

      if (!response.ok) {
        throw new Error(`Response status: ${response.status}`);
      }

      const resultText = await response.text();
      if (resultText.trim() === "") {
        throw new Error("Empty response from server");
      }

      const result = JSON.parse(resultText);
      console.log("Network settings updated:", result);
    } catch (error) {
      console.error("Failed to update network settings:", error.message);
    }
  });

async function fetchIpAddress() {
  try {
    const response = await fetch("/ip");
    if (!response.ok) {
      throw new Error(`Response status: ${response.status}`);
    }

    console.log(response);
    const json = await response.json();
    console.log(json);
    const ipAddressElement = document.getElementById("currentIp");
    ipAddressElement.innerHTML = json.ip_address;
  } catch (error) {
    console.error(error.message);
  }
}

async function fetchNamespace() {
  try {
    const response = await fetch("/namespace");
    if (!response.ok) {
      throw new Error(`Response status: ${response.status}`);
    }

    const json = await response.json();
    const namespaceElement = document.getElementById("currentNamespace");
    namespaceElement.innerHTML = json.namespace;
  } catch (error) {
    console.error(error.message);
  }
}

async function fetchDomainId() {
  try {
    const response = await fetch("/domain_id");
    if (!response.ok) {
      throw new Error(`Response status: ${response.status}`);
    }

    const json = await response.json();
    const domainIdElement = document.getElementById("currentDomainId");
    domainIdElement.innerHTML = json.domain_id;
  } catch (error) {
    console.error(error.message);
  }
}

window.addEventListener("DOMContentLoaded", async (ev) => {
  /* Fetch the uptime every 10 seconds */
  setInterval(fetchUptime, 10000);

  /* Fetch the version once */
  await fetchVersion();
  /* Fetch the SSID once */
  await fetchSSID();
  /* Fetch the current IP address once */
  await fetchIpAddress();
  /* Fetch the current namespace once */
  await fetchNamespace();
  /* Fetch the current domain ID once */
  await fetchDomainId();
});
