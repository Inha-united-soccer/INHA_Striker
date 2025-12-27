#pragma once

#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vision_interface/msg/detections.hpp>
#include "types.h"

// 전방선언
class BrainConfig;
class BrainData;
class BrainLog;
class BrainTree;

/*
    해당 파일은 Vision에서 감지된 객체들을 게임 객체로 변환하고,
    감지된 객체들을 후처리하는 함수들만 모아놨습니다.
*/


namespace detection_utils {

// 시간 관련 함수
rclcpp::Time timePointFromHeader(const std_msgs::msg::Header &header);

// 감지된 객체들을 게임 객체로 변환
std::vector<GameObject> detectionsToGameObjects(const vision_interface::msg::Detections &detections, const std::shared_ptr<BrainConfig> &config, const std::shared_ptr<BrainData> &data);

/* ----------------------------- Ball 전처리 ----------------------------- */
// 감지된 공 객체들을 처리
void detectProcessBalls(const std::vector<GameObject> &ballObjs, const std::shared_ptr<BrainConfig> &config, const std::shared_ptr<BrainData> &data, const std::shared_ptr<BrainTree> &tree);

// 공이 필드 밖으로 나갔는지 판단
bool isBallOut(double locCompareDist, double lineCompareDist, const std::shared_ptr<BrainConfig> &config, const std::shared_ptr<BrainData> &data);
void updateBallOut(const std::shared_ptr<BrainConfig> &config, const std::shared_ptr<BrainData> &data, const std::shared_ptr<BrainTree> &tree);


/* ----------------------------- Localization을 위한 마커, 라인 전처리 ----------------------------- */
// 필드 라인 필터링 함수
void updateLinePosToField(FieldLine& line, const std::shared_ptr<BrainData> &data);
vector<FieldLine> processFieldLines(vector<FieldLine>& fieldLines, const std::shared_ptr<BrainConfig> &config, const std::shared_ptr<BrainData> &data, const std::shared_ptr<BrainTree> &tree);
void identifyFieldLine(FieldLine& line, const std::shared_ptr<BrainConfig> &config, const std::shared_ptr<BrainData> &data, const std::shared_ptr<BrainTree> &tree);

// 마킹 개수 계산
int markCntOnFieldLine(const string markType, const FieldLine line, const std::shared_ptr<BrainData> &data, const double margin=0.2);
// 골포스트 개수 계산
int goalpostCntOnFieldLine(const FieldLine line, const std::shared_ptr<BrainData> &data, const double margin=0.2);
// 공이 특정 라인 위에 있는지 판단
bool isBallOnFieldLine(const FieldLine line, const std::shared_ptr<BrainData> &data, const double margin=0.3);

// 마커, 라인 처리
void detectProcessMarkings(const vector<GameObject> &markingObjs, const std::shared_ptr<BrainData> &data, const std::shared_ptr<BrainConfig> &config, const std::shared_ptr<BrainLog> &log);
void detectProcessGoalposts(const vector<GameObject> &goalpostObjs, const std::shared_ptr<BrainData> &data, const std::shared_ptr<BrainLog> &log);
void detectProcessVisionBox(const vision_interface::msg::Detections &msg, const std::shared_ptr<BrainData> &data);
void detectProcessRobots(const vector<GameObject> &robotObjs, const std::shared_ptr<BrainData> &data, const std::shared_ptr<BrainConfig> &config);

// 마커, 라인 식별
void identifyGoalpost(GameObject& goalpost);
void identifyMarking(GameObject& marking, const std::shared_ptr<BrainConfig> &config);
void identifyTeammates(std::vector<GameObject>& robots, const std::shared_ptr<BrainData> &data);



}