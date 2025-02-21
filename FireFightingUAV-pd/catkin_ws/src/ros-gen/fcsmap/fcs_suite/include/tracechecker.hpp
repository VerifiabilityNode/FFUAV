
#ifndef _H_TRACECHECKER
#define _H_TRACECHECKER

class TraceChecker {
  protected:
    bool valid() {
      return it < end;
    }
  public:
    TraceChecker();
    TraceChecker(std::vector<std::shared_ptr<Message>>::iterator _it,
                 std::vector<std::shared_ptr<Message>>::iterator _end,
                 Verdict _verdict);
    TraceChecker initially(std::vector<std::shared_ptr<Message>> vec);
    TraceChecker then(std::shared_ptr<Event> ev);
    TraceChecker pass(std::shared_ptr<Event> ev);
    TraceChecker pass();
    TraceChecker forbid(std::shared_ptr<Event> ev);
    Verdict getVerdict();

    std::vector<std::shared_ptr<Message>>::iterator it;
    std::vector<std::shared_ptr<Message>>::iterator end;
    Verdict verdict;
};

#endif /* _H_TRACECHECKER */
