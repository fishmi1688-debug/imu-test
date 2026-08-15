model = "gpt-5.5"
model_reasoning_effort = "xhigh"
personality = "pragmatic"
service_tier = "default"
[features]
multi_agent = true

[projects."/media/zhang/新加卷/wc/test/emg-test/gait_control_ws"]
trust_level = "trusted"

[projects."/media/zhang/新加卷/wc/PC-test"]
trust_level = "trusted"

[projects."/media/zhang/新加卷/wc/test/new-dianji/PC-test"]
trust_level = "trusted"

[projects."/media/zhang/新加卷/wc/test/imu+model+trian/步态模型"]
trust_level = "trusted"

[projects."/media/zhang/新加卷/wc/emg"]
trust_level = "trusted"

[projects."/media/zhang/新加卷/wc/test/多个相位生成测试/PC-test"]
trust_level = "trusted"

[projects."/media/zhang/新加卷/wc/test/灵足/PC-test"]
trust_level = "trusted"

[projects."/media/zhang/新加卷/wc/test/imu-test/PC-test"]
trust_level = "trusted"

[projects."/media/zhang/新加卷/wc/test/imu+model+trian"]
trust_level = "trusted"





model_provider = "codex-for-me"
model = "gpt-5.4"
model_reasoning_effort = "high"

[model_providers.codex-for-me]
name = "openai"
base_url = "https://blackaicoding.com/v1"
wire_api = "responses"
requires_openai_auth = true

[features]
goals = true